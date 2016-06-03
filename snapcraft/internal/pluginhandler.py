# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 3 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import contextlib
import filecmp
import glob
import importlib
import itertools
import logging
import os
import shutil
import sys

import jsonschema
import magic
import yaml

import snapcraft
from snapcraft.internal import (
    common,
    libraries,
    repo,
    states,
)

_SNAPCRAFT_STAGE = '$SNAPCRAFT_STAGE'

logger = logging.getLogger(__name__)


class PluginError(Exception):
    pass


class MissingState(Exception):
    pass


class PluginHandler:

    @property
    def name(self):
        return self._name

    @property
    def installdir(self):
        return self.code.installdir

    @property
    def ubuntu(self):
        if not self._ubuntu:
            self._ubuntu = repo.Ubuntu(
                self.ubuntudir, sources=self.code.PLUGIN_STAGE_SOURCES,
                project_options=self._project_options)

        return self._ubuntu

    def __init__(self, plugin_name, part_name, properties,
                 project_options, part_schema):
        self.valid = False
        self.code = None
        self.config = {}
        self._name = part_name
        self._ubuntu = None
        self._project_options = project_options
        self.deps = []

        self.stagedir = project_options.stage_dir
        self.snapdir = project_options.snap_dir

        parts_dir = project_options.parts_dir
        self.bindir = os.path.join(parts_dir, part_name, 'bin')
        self.ubuntudir = os.path.join(parts_dir, part_name, 'ubuntu')
        self.statedir = os.path.join(parts_dir, part_name, 'state')

        self._migrate_state_file()

        try:
            self._load_code(plugin_name, properties, part_schema)
        except jsonschema.ValidationError as e:
            raise PluginError('properties failed to load for {}: {}'.format(
                part_name, e.message))

    def _load_code(self, plugin_name, properties, part_schema):
        module_name = plugin_name.replace('-', '_')
        module = None

        with contextlib.suppress(ImportError):
            module = _load_local('x-{}'.format(plugin_name),
                                 self._project_options.local_plugins_dir)
            logger.info('Loaded local plugin for %s', plugin_name)

        if not module:
            with contextlib.suppress(ImportError):
                module = importlib.import_module(
                    'snapcraft.plugins.{}'.format(module_name))

        if not module:
            logger.info('Searching for local plugin for %s', plugin_name)
            with contextlib.suppress(ImportError):
                module = _load_local(module_name,
                                     self._project_options.local_plugins_dir)
            if not module:
                raise PluginError('unknown plugin: {}'.format(plugin_name))

        plugin = _get_plugin(module)
        options, self.pull_properties, self.build_properties = _make_options(
            self._project_options.stage_dir, part_schema, properties,
            plugin.schema())
        # For backwards compatibility we add the project to the plugin
        try:
            self.code = plugin(self.name, options, self._project_options)
        except TypeError:
            logger.warning(
                'DEPRECATED: the plugin used by part {!r} needs to be updated '
                'to accept project options in its initializer. See '
                'https://github.com/ubuntu-core/snapcraft/blob/master/docs/'
                'plugins.md#initializing-a-plugin for more information'.format(
                    self.name))
            self.code = plugin(self.name, options)
            # This is for plugins that don't inherit from BasePlugin
            if not hasattr(self.code, 'project'):
                setattr(self.code, 'project', self._project_options)
            # This is for plugins that inherit from BasePlugin but don't have
            # project in init.
            if not self.code.project:
                self.code.project = self._project_options

        if self._project_options.is_cross_compiling:
            logger.debug(
                'Setting {!r} as the compilation target for {!r}'.format(
                    self._project_options.deb_arch, plugin_name))
            self.code.enable_cross_compilation()

    def makedirs(self):
        dirs = [
            self.code.sourcedir, self.code.builddir, self.code.installdir,
            self.stagedir, self.snapdir, self.ubuntudir, self.statedir
        ]
        for d in dirs:
            os.makedirs(d, exist_ok=True)

    def _migrate_state_file(self):
        # In previous versions of Snapcraft, the state directory was a file.
        # Rather than die if we're running on output from an old version,
        # migrate it for them.
        if os.path.isfile(self.statedir):
            with open(self.statedir, 'r') as f:
                step = f.read()

            if step:
                os.remove(self.statedir)
                os.makedirs(self.statedir)
                self.mark_done(step)

    def notify_part_progress(self, progress, hint=''):
        logger.info('%s %s %s', progress, self.name, hint)

    def last_step(self):
        for step in reversed(common.COMMAND_ORDER):
            if os.path.exists(self._step_state_file(step)):
                return step

        return None

    def is_clean(self, step):
        """Return true if the given step hasn't run (or has been cleaned)."""

        last_step = self.last_step()
        if last_step:
            return (common.COMMAND_ORDER.index(step) >
                    common.COMMAND_ORDER.index(last_step))

        return True

    def is_dirty(self, step):
        """Return true if the given step needs to run again."""

        # Retrieve the stored state for this step (assuming it has already run)
        state = self.get_state(step)
        with contextlib.suppress(AttributeError):
            # state.properties contains the old YAML that this step cares
            # about, and we're comparing it to those same keys in the current
            # YAML (taken from self.code.options). If they've changed, then
            # this step is dirty and needs to run again.
            if state.properties != state.properties_of_interest(
                    self.code.options):
                return True

        with contextlib.suppress(AttributeError):
            # state.project_options contains the old project options that this
            # step cares about, and we're comparing it to those same options in
            # the current project. If they've changed, then this step is dirty
            # and needs to run again.
            if state.project_options != state.project_options_of_interest(
                    self._project_options):
                return True

        return False

    def should_step_run(self, step, force=False):
        return force or self.is_clean(step)

    def mark_done(self, step, state=None):
        if not state:
            state = {}

        index = common.COMMAND_ORDER.index(step)

        with open(self._step_state_file(step), 'w') as f:
            f.write(yaml.dump(state))

        # We know we've only just completed this step, so make sure any later
        # steps don't have a saved state.
        if index+1 != len(common.COMMAND_ORDER):
            for command in common.COMMAND_ORDER[index+1:]:
                self.mark_cleaned(command)

    def mark_cleaned(self, step):
        state_file = self._step_state_file(step)
        if os.path.exists(state_file):
            os.remove(state_file)

        if os.path.isdir(self.statedir) and not os.listdir(self.statedir):
            os.rmdir(self.statedir)

    def get_state(self, step):
        state = None
        state_file = self._step_state_file(step)
        if os.path.isfile(state_file):
            with open(state_file, 'r') as f:
                state = yaml.load(f.read())

        return state

    def _step_state_file(self, step):
        return os.path.join(self.statedir, step)

    def _fetch_stage_packages(self):
        if not self.code.stage_packages:
            return

        try:
            self.ubuntu.get(self.code.stage_packages)
        except repo.PackageNotFoundError as e:
            raise RuntimeError("Error downloading stage packages for part "
                               "{!r}: no such package {!r}".format(
                                   self.name, e.package_name))

    def _unpack_stage_packages(self):
        if self.code.stage_packages:
            self.ubuntu.unpack(self.installdir)

    def prepare_pull(self, force=False):
        self.makedirs()
        self.notify_part_progress('Preparing to pull')
        self._fetch_stage_packages()
        self._unpack_stage_packages()

    def pull(self, force=False):
        self.makedirs()
        self.notify_part_progress('Pulling')
        self.code.pull()
        self.mark_done('pull', states.PullState(
            self.pull_properties, self.code.options, self._project_options))

    def clean_pull(self, hint=''):
        if self.is_clean('pull'):
            hint = '{} {}'.format(hint, '(already clean)').strip()
            self.notify_part_progress('Skipping cleaning pulled source for',
                                      hint)
            return

        self.notify_part_progress('Cleaning pulled source for', hint)
        # Remove ubuntu cache (where stage packages are fetched)
        if os.path.exists(self.ubuntudir):
            shutil.rmtree(self.ubuntudir)

        self.code.clean_pull()
        self.mark_cleaned('pull')

    def prepare_build(self, force=False):
        self.makedirs()
        self.notify_part_progress('Preparing to build')
        # Stage packages are fetched and unpacked in the pull step, but we'll
        # unpack again here just in case the build step has been cleaned.
        self._unpack_stage_packages()

    def build(self, force=False):
        self.makedirs()
        self.notify_part_progress('Building')
        self.code.build()
        self.mark_done('build', states.BuildState(
            self.build_properties, self.code.options, self._project_options))

    def clean_build(self, hint=''):
        if self.is_clean('build'):
            hint = '{} {}'.format(hint, '(already clean)').strip()
            self.notify_part_progress('Skipping cleaning build for',
                                      hint)
            return

        self.notify_part_progress('Cleaning build for', hint)
        self.code.clean_build()
        self.mark_cleaned('build')

    def migratable_fileset_for(self, step):
        plugin_fileset = self.code.snap_fileset()
        fileset = (getattr(self.code.options, step, ['*']) or ['*']).copy()
        fileset.extend(plugin_fileset)

        return _migratable_filesets(fileset, self.code.installdir)

    def _organize(self):
        organize_fileset = getattr(self.code.options, 'organize', {}) or {}

        for key in organize_fileset:
            src = os.path.join(self.code.installdir, key)
            dst = os.path.join(self.code.installdir, organize_fileset[key])

            os.makedirs(os.path.dirname(dst), exist_ok=True)

            if os.path.exists(dst):
                logger.warning(
                    'Stepping over existing file for organization %r',
                    os.path.relpath(dst, self.code.installdir))
                if os.path.isdir(dst):
                    shutil.rmtree(dst)
                else:
                    os.remove(dst)
            shutil.move(src, dst)

    def stage(self, force=False):
        self.makedirs()
        self.notify_part_progress('Staging')
        self._organize()
        snap_files, snap_dirs = self.migratable_fileset_for('stage')
        _migrate_files(snap_files, snap_dirs, self.code.installdir,
                       self.stagedir)
        # TODO once `snappy try` is in place we will need to copy
        # dependencies here too

        self.mark_done('stage', states.StageState(
            snap_files, snap_dirs, self.code.options, self._project_options))

    def clean_stage(self, project_staged_state, hint=''):
        if self.is_clean('stage'):
            hint = '{} {}'.format(hint, '(already clean)').strip()
            self.notify_part_progress('Skipping cleaning staging area for',
                                      hint)
            return

        self.notify_part_progress('Cleaning staging area for', hint)

        state = self.get_state('stage')

        try:
            self._clean_shared_area(self.stagedir, state,
                                    project_staged_state)
        except AttributeError:
            raise MissingState(
                "Failed to clean step 'stage': Missing necessary state. "
                "This won't work until a complete clean has occurred.")

        self.mark_cleaned('stage')

    def prime(self, force=False):
        self.makedirs()
        self.notify_part_progress('Priming')
        snap_files, snap_dirs = self.migratable_fileset_for('snap')
        _migrate_files(snap_files, snap_dirs, self.stagedir, self.snapdir)
        dependencies = _find_dependencies(self.snapdir)

        # Split the necessary dependencies into their corresponding location.
        # We'll both migrate and track the system dependencies, but we'll only
        # track the part and staged dependencies, since they should have
        # already been primed by other means, and migrating them again could
        # potentially override the `stage` or `snap` filtering.
        part_dependencies = set()
        staged_dependencies = set()
        system_dependencies = set()
        for file_path in dependencies:
            if file_path.startswith(self.installdir):
                part_dependencies.add(
                    os.path.relpath(file_path, self.installdir))
            elif file_path.startswith(self.stagedir):
                staged_dependencies.add(
                    os.path.relpath(file_path, self.stagedir))
            else:
                system_dependencies.add(file_path.lstrip('/'))

        part_dependency_paths = {os.path.dirname(d) for d in part_dependencies}
        staged_dependency_paths = {os.path.dirname(d) for d in
                                   staged_dependencies}
        system_dependency_paths = {os.path.dirname(d) for d in
                                   system_dependencies}
        # Lots of dependencies are linked with a symlink, so we need to make
        # sure we follow those symlinks when we migrate the dependencies.
        _migrate_files(system_dependencies, system_dependency_paths, '/',
                       self.snapdir, follow_symlinks=True)

        dependency_paths = (part_dependency_paths | staged_dependency_paths |
                            system_dependency_paths)
        self.mark_done('prime', states.PrimeState(
            snap_files, snap_dirs, dependency_paths, self.code.options,
            self._project_options))

    def clean_prime(self, project_primed_state, hint=''):
        if self.is_clean('prime'):
            hint = '{} {}'.format(hint, '(already clean)').strip()
            self.notify_part_progress('Skipping cleaning priming area for',
                                      hint)
            return

        self.notify_part_progress('Cleaning priming area for', hint)

        state = self.get_state('prime')

        try:
            self._clean_shared_area(self.snapdir, state,
                                    project_primed_state)
        except AttributeError:
            raise MissingState(
                "Failed to clean step 'prime': Missing necessary state. "
                "This won't work until a complete clean has occurred.")

        self.mark_cleaned('prime')

    def _clean_shared_area(self, shared_directory, part_state, project_state):
        primed_files = part_state.files
        primed_directories = part_state.directories

        # We want to make sure we don't remove a file or directory that's
        # being used by another part. So we'll examine the state for all parts
        # in the project and leave any files or directories found to be in
        # common.
        for other_name, other_state in project_state.items():
            if other_state and (other_name != self.name):
                primed_files -= other_state.files
                primed_directories -= other_state.directories

        # Finally, clean the files and directories that are specific to this
        # part.
        _clean_migrated_files(primed_files, primed_directories,
                              shared_directory)

    def get_primed_dependency_paths(self):
        dependency_paths = set()
        state = self.get_state('prime')
        if state:
            for path in state.dependency_paths:
                dependency_paths.add(
                    os.path.join(self.snapdir, path.lstrip('/')))

        return dependency_paths

    def env(self, root):
        return self.code.env(root)

    def clean(self, project_staged_state=None, project_primed_state=None,
              step=None, hint=''):
        if not project_staged_state:
            project_staged_state = {}

        if not project_primed_state:
            project_primed_state = {}

        try:
            self._clean_steps(project_staged_state, project_primed_state,
                              step, hint)
        except MissingState:
            # If one of the step cleaning rules is missing state, it must be
            # running on the output of an old Snapcraft. In that case, if we
            # were specifically asked to clean that step we need to fail.
            # Otherwise, just clean like the old Snapcraft did, and blow away
            # the entire part directory.
            if step:
                raise

            logger.info('Cleaning up for part {!r}'.format(self.name))
            if os.path.exists(self.code.partdir):
                shutil.rmtree(self.code.partdir)

        # Remove the part directory if it's completely empty (i.e. all steps
        # have been cleaned).
        if (os.path.exists(self.code.partdir) and
                not os.listdir(self.code.partdir)):
            os.rmdir(self.code.partdir)

    def _clean_steps(self, project_staged_state, project_primed_state,
                     step=None, hint=None):
        index = None
        if step:
            if step not in common.COMMAND_ORDER:
                raise RuntimeError(
                    '{!r} is not a valid step for part {!r}'.format(
                        step, self.name))

            index = common.COMMAND_ORDER.index(step)

        if not index or index <= common.COMMAND_ORDER.index('prime'):
            self.clean_prime(project_primed_state, hint)

        if not index or index <= common.COMMAND_ORDER.index('stage'):
            self.clean_stage(project_staged_state, hint)

        if not index or index <= common.COMMAND_ORDER.index('build'):
            self.clean_build(hint)

        if not index or index <= common.COMMAND_ORDER.index('pull'):
            self.clean_pull(hint)


def _validate_step_properties(step, plugin_schema):
    step_properties_key = '{}-properties'.format(step)
    properties = plugin_schema.get('properties', {})
    step_properties = plugin_schema.get(step_properties_key, [])

    invalid_properties = set()
    for step_property in step_properties:
        if step_property not in properties:
            invalid_properties.add(step_property)

    if invalid_properties:
        raise jsonschema.exceptions.ValidationError(
            "Invalid {} specified in plugin's schema: {}".format(
                step_properties_key, list(invalid_properties)))


def _make_options(stage_dir, part_schema, properties, plugin_schema):
    if 'properties' not in plugin_schema:
        plugin_schema['properties'] = {}
    # The base part_schema takes precedense over the plugin.
    plugin_schema['properties'].update(part_schema)

    _validate_step_properties('pull', plugin_schema)
    _validate_step_properties('build', plugin_schema)
    jsonschema.validate(properties, plugin_schema)

    class Options():
        pass
    options = Options()

    _populate_options(stage_dir, options, properties, plugin_schema)

    return (options, plugin_schema.get('pull-properties', []),
            plugin_schema.get('build-properties', []))


def _populate_options(stage_dir, options, properties, schema):
    schema_properties = schema.get('properties', {})
    for key in schema_properties:
        attr_name = key.replace('-', '_')
        default_value = schema_properties[key].get('default')
        attr_value = _expand_env(properties.get(key, default_value), stage_dir)
        setattr(options, attr_name, attr_value)


def _expand_env(attr, stage_dir):
    if isinstance(attr, str) and _SNAPCRAFT_STAGE in attr:
        return attr.replace(_SNAPCRAFT_STAGE, stage_dir)
    elif isinstance(attr, list) or isinstance(attr, tuple):
        return [_expand_env(i, stage_dir) for i in attr]
    elif isinstance(attr, dict):
        return {k: _expand_env(attr[k], stage_dir) for k in attr}

    return attr


def _get_plugin(module):
    for attr in vars(module).values():
        if not isinstance(attr, type):
            continue
        if not issubclass(attr, snapcraft.BasePlugin):
            continue
        if attr == snapcraft.BasePlugin:
            continue
        return attr


def _load_local(module_name, local_plugin_dir):
    sys.path = [local_plugin_dir] + sys.path
    module = importlib.import_module(module_name)
    sys.path.pop(0)

    return module


def load_plugin(part_name, plugin_name, properties=None,
                project_options=None, part_schema=None):
    if properties is None:
        properties = {}
    if part_schema is None:
        part_schema = {}
    if project_options is None:
        project_options = snapcraft.ProjectOptions()
    return PluginHandler(plugin_name, part_name, properties,
                         project_options, part_schema)


def _migratable_filesets(fileset, srcdir):
    includes, excludes = _get_file_list(fileset)

    include_files = _generate_include_set(srcdir, includes)
    exclude_files, exclude_dirs = _generate_exclude_set(srcdir, excludes)

    # And chop files, including whole trees if any dirs are mentioned
    snap_files = include_files - exclude_files
    for exclude_dir in exclude_dirs:
        snap_files = set([x for x in snap_files
                          if not x.startswith(exclude_dir + '/')])

    # Separate dirs from files
    snap_dirs = set([x for x in snap_files
                     if os.path.isdir(os.path.join(srcdir, x)) and
                     not os.path.islink(os.path.join(srcdir, x))])
    snap_files = snap_files - snap_dirs

    # Make sure we also obtain the parent directories of files
    for snap_file in snap_files:
        dirname = os.path.dirname(snap_file)
        while dirname:
            snap_dirs.add(dirname)
            dirname = os.path.dirname(dirname)

    return snap_files, snap_dirs


def _migrate_files(snap_files, snap_dirs, srcdir, dstdir, missing_ok=False,
                   follow_symlinks=False):
    for directory in snap_dirs:
        os.makedirs(os.path.join(dstdir, directory), exist_ok=True)

    for snap_file in snap_files:
        src = os.path.join(srcdir, snap_file)
        dst = os.path.join(dstdir, snap_file)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        if missing_ok and not os.path.exists(src):
            continue

        # If the file is already here and it's a symlink, leave it alone.
        if os.path.islink(dst):
            continue

        # Otherwise, remove and re-link it.
        if os.path.exists(dst):
            os.remove(dst)

        common.link_or_copy(src, dst, follow_symlinks=follow_symlinks)


def _clean_migrated_files(snap_files, snap_dirs, directory):
    for snap_file in snap_files:
        os.remove(os.path.join(directory, snap_file))

    # snap_dirs may not be ordered so that subdirectories come before
    # parents, and we want to be able to remove directories if possible, so
    # we'll sort them in reverse here to get subdirectories before parents.
    snap_dirs = sorted(snap_dirs, reverse=True)

    for snap_dir in snap_dirs:
        migrated_directory = os.path.join(directory, snap_dir)
        if not os.listdir(migrated_directory):
            os.rmdir(migrated_directory)


def _find_dependencies(workdir):
    ms = magic.open(magic.NONE)
    if ms.load() != 0:
        raise RuntimeError('Cannot load magic header detection')

    elf_files = set()
    fs_encoding = sys.getfilesystemencoding()

    for root, dirs, files in os.walk(workdir.encode(fs_encoding)):
        # Filter out object (*.o) files-- we only care about binaries.
        entries = (entry for entry in itertools.chain(files, dirs)
                   if not entry.endswith(b'.o'))
        for entry in entries:
            path = os.path.join(root, entry)
            if os.path.islink(path):
                logger.debug('Skipped link {!r} when parsing {!r}'.format(
                    path, workdir))
                continue
            file_m = ms.file(path)
            if file_m.startswith('ELF') and 'dynamically linked' in file_m:
                elf_files.add(path)

    dependencies = []
    for elf_file in elf_files:
        dependencies += libraries.get_dependencies(elf_file)

    return set(dependencies)


def _get_file_list(stage_set):
    includes = []
    excludes = []

    for item in stage_set:
        if item.startswith('-'):
            excludes.append(item[1:])
        elif item.startswith('\\'):
            includes.append(item[1:])
        else:
            includes.append(item)

    _validate_relative_paths(includes + excludes)

    includes = includes or ['*']

    return includes, excludes


def _generate_include_set(directory, includes):
    include_files = set()
    for include in includes:
        if '*' in include:
            matches = glob.glob(os.path.join(directory, include))
            include_files |= set(matches)
        else:
            include_files |= set([os.path.join(directory, include), ])

    include_dirs = [x for x in include_files if os.path.isdir(x)]
    include_files = set([os.path.relpath(x, directory) for x in include_files])

    # Expand includeFiles, so that an exclude like '*/*.so' will still match
    # files from an include like 'lib'
    for include_dir in include_dirs:
        for root, dirs, files in os.walk(include_dir):
            include_files |= \
                set([os.path.relpath(os.path.join(root, d), directory)
                     for d in dirs])
            include_files |= \
                set([os.path.relpath(os.path.join(root, f), directory)
                     for f in files])

    return include_files


def _generate_exclude_set(directory, excludes):
    exclude_files = set()

    for exclude in excludes:
        matches = glob.glob(os.path.join(directory, exclude))
        exclude_files |= set(matches)

    exclude_dirs = [os.path.relpath(x, directory)
                    for x in exclude_files if os.path.isdir(x)]
    exclude_files = set([os.path.relpath(x, directory)
                         for x in exclude_files])

    return exclude_files, exclude_dirs


def _validate_relative_paths(files):
    for d in files:
        if os.path.isabs(d):
            raise PluginError('path "{}" must be relative'.format(d))


def check_for_collisions(parts):
    """Raises an EnvironmentError if conflicts are found between two parts."""
    parts_files = {}
    for part in parts:
        # Gather our own files up
        part_files, _ = part.migratable_fileset_for('stage')

        # Scan previous parts for collisions
        for other_part_name in parts_files:
            common = part_files & parts_files[other_part_name]['files']
            conflict_files = []
            for f in common:
                this = os.path.join(part.installdir, f)
                other = os.path.join(
                    parts_files[other_part_name]['installdir'],
                    f)
                if os.path.islink(this) and os.path.islink(other):
                    continue
                if not filecmp.cmp(this, other, shallow=False):
                    conflict_files.append(f)

            if conflict_files:
                raise EnvironmentError(
                    'Parts {!r} and {!r} have the following file paths in '
                    'common which have different contents:\n{}'.format(
                        other_part_name, part.name,
                        '\n'.join(sorted(conflict_files))))

        # And add our files to the list
        parts_files[part.name] = {'files': part_files,
                                  'installdir': part.installdir}
