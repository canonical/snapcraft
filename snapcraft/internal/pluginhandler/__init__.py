# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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
import copy
import filecmp
import importlib
import logging
import os
import shutil
import sys
from glob import glob, iglob

import jsonschema
import magic
import yaml

import snapcraft
from snapcraft import file_utils
from snapcraft.internal.errors import (
    PrimeFileConflictError,
    PluginError,
    MissingState,
    SnapcraftPartConflictError,
    SnapcraftSchemaError
)
from snapcraft.internal import (
    common,
    libraries,
    repo,
    sources,
    states,
)
from ._scriptlets import ScriptRunner
from ._build_attributes import BuildAttributes
from ._stage_package_handler import StagePackageHandler

logger = logging.getLogger(__name__)


class DirtyReport:
    def __init__(self, dirty_properties, dirty_project_options):
        self.dirty_properties = dirty_properties
        self.dirty_project_options = dirty_project_options


class PluginHandler:

    @property
    def name(self):
        return self._name

    @property
    def installdir(self):
        return self.code.installdir

    def __init__(self, *, plugin_name, part_name,
                 part_properties, project_options, part_schema,
                 definitions_schema):
        self.valid = False
        self.code = None
        self.config = {}
        self._name = part_name
        self._part_properties = _expand_part_properties(
            part_properties, part_schema)
        self.stage_packages = []
        self.build_packages = []

        # Some legacy parts can have a '/' in them to separate the main project
        # part with the subparts. This is rather unfortunate as it affects the
        # the layout of parts inside the parts directory causing collisions
        # between the main project part and its subparts.
        part_name = part_name.replace('/', '\N{BIG SOLIDUS}')
        self._project_options = project_options
        self.deps = []

        self.stagedir = project_options.stage_dir
        self.snapdir = project_options.snap_dir

        parts_dir = project_options.parts_dir
        self.ubuntudir = os.path.join(parts_dir, part_name, 'ubuntu')
        self.statedir = os.path.join(parts_dir, part_name, 'state')
        self.sourcedir = os.path.join(parts_dir, part_name, 'src')

        self.source_handler = self._get_source_handler(self._part_properties)

        self._build_attributes = BuildAttributes(
            self._part_properties['build-attributes'])

        self._migrate_state_file()

        try:
            self._load_code(
                plugin_name, self._part_properties, part_schema,
                definitions_schema)
        except jsonschema.ValidationError as e:
            error = SnapcraftSchemaError.from_validation_error(e)
            raise PluginError('properties failed to load for {}: {}'.format(
                part_name, error.message))

        stage_packages = getattr(self.code, 'stage_packages', [])
        sources = getattr(self.code, 'PLUGIN_STAGE_SOURCES', None)
        self._stage_package_handler = StagePackageHandler(
            stage_packages, self.ubuntudir,
            sources=sources, project_options=self._project_options)

    def _load_code(self, plugin_name, properties, part_schema,
                   definitions_schema):
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
        _validate_pull_and_build_properties(
            plugin_name, plugin, part_schema, definitions_schema)
        options = _make_options(
            part_schema, definitions_schema, properties, plugin.schema())
        # For backwards compatibility we add the project to the plugin
        try:
            self.code = plugin(self.name, options, self._project_options)
        except TypeError:
            logger.warning(
                'DEPRECATED: the plugin used by part {!r} needs to be updated '
                'to accept project options in its initializer. See '
                'https://github.com/snapcore/snapcraft/blob/master/docs/'
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

    def _get_source_handler(self, properties):
        """Returns a source_handler for the source in properties."""
        # TODO: we cannot pop source as it is used by plugins. We also make
        # the default '.'
        source_handler = None
        if properties['source']:
            handler_class = sources.get_source_handler(
                properties['source'], source_type=properties['source-type'])
            source_handler = handler_class(
                properties['source'],
                self.sourcedir,
                source_checksum=properties['source-checksum'],
                source_branch=properties['source-branch'],
                source_tag=properties['source-tag'],
                source_depth=properties['source-depth'],
                source_commit=properties['source-commit'],
            )

        return source_handler

    def makedirs(self):
        dirs = [
            self.code.sourcedir, self.code.builddir, self.code.installdir,
            self.stagedir, self.snapdir, self.statedir
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

        return self.get_dirty_report(step) is not None

    def get_dirty_report(self, step):
        """Return a DirtyReport class describing why step is dirty.

        Returns None if step is not dirty.
        """

        # Retrieve the stored state for this step (assuming it has already run)
        state = self.get_state(step)
        differing_properties = set()
        differing_options = set()

        with contextlib.suppress(AttributeError):
            # state.properties contains the old YAML that this step cares
            # about, and we're comparing it to those same keys in the current
            # YAML (self._part_properties). If they've changed, then this step
            # is dirty and needs to run again.
            differing_properties = state.diff_properties_of_interest(
                self._part_properties)

        with contextlib.suppress(AttributeError):
            # state.project_options contains the old project options that this
            # step cares about, and we're comparing it to those same options in
            # the current project. If they've changed, then this step is dirty
            # and needs to run again.
            differing_options = state.diff_project_options_of_interest(
                self._project_options)

        if differing_properties or differing_options:
            return DirtyReport(differing_properties, differing_options)

        return None

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
        try:
            self.stage_packages = self._stage_package_handler.fetch()
        except repo.errors.PackageNotFoundError as e:
            raise RuntimeError("Error downloading stage packages for part "
                               "{!r}: {}".format(self.name, e.message))

    def _unpack_stage_packages(self):
        self._stage_package_handler.unpack(self.installdir)

    def prepare_pull(self, force=False):
        self.makedirs()
        self.notify_part_progress('Preparing to pull')
        self._fetch_stage_packages()
        self._unpack_stage_packages()

    def pull(self, force=False):
        self.makedirs()
        self.notify_part_progress('Pulling')
        if self.source_handler:
            self.source_handler.pull()
        self.code.pull()

        self.mark_pull_done()

    def mark_pull_done(self):
        pull_properties = self.code.get_pull_properties()

        # Add the annotated list of build packages
        part_build_packages = self._part_properties.get('build-packages', [])
        build_packages = repo.Repo.get_installed_build_packages(
            part_build_packages)
        versioned_build_packages = []
        for pkg in build_packages:
            if pkg in part_build_packages:
                versioned_build_packages.append(pkg)
            else:
                pkg_name, version = repo.get_pkg_name_parts(pkg)
                if pkg_name in part_build_packages:
                    versioned_build_packages.append(pkg)

        self.mark_done('pull', states.PullState(
            pull_properties, part_properties=self._part_properties,
            project=self._project_options, stage_packages=self.stage_packages,
            build_packages=versioned_build_packages,
        ))

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

        if os.path.exists(self.sourcedir):
            if os.path.islink(self.sourcedir):
                os.remove(self.sourcedir)
            else:
                shutil.rmtree(self.sourcedir)

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

        if os.path.exists(self.code.build_basedir):
            shutil.rmtree(self.code.build_basedir)

        # FIXME: It's not necessary to ignore here anymore since it's now done
        # in the Local source. However, it's left here so that it continues to
        # work on old snapcraft trees that still have src symlinks.
        def ignore(directory, files):
            if directory == self.sourcedir:
                snaps = glob(os.path.join(directory, '*.snap'))
                if snaps:
                    snaps = [os.path.basename(s) for s in snaps]
                    return common.SNAPCRAFT_FILES + snaps
                else:
                    return common.SNAPCRAFT_FILES
            else:
                return []

        shutil.copytree(self.code.sourcedir, self.code.build_basedir,
                        symlinks=True, ignore=ignore)

        script_runner = ScriptRunner(builddir=self.code.build_basedir)

        script_runner.run(scriptlet=self._part_properties.get('prepare'))
        build_scriptlet = self._part_properties.get('build')
        if build_scriptlet:
            script_runner.run(scriptlet=build_scriptlet)
        else:
            self.code.build()
        script_runner.run(scriptlet=self._part_properties.get('install'))

        self.mark_build_done()

    def mark_build_done(self):
        build_properties = self.code.get_build_properties()

        self.mark_done('build', states.BuildState(
            build_properties, self._part_properties,
            self._project_options))

    def clean_build(self, hint=''):
        if self.is_clean('build'):
            hint = '{} {}'.format(hint, '(already clean)').strip()
            self.notify_part_progress('Skipping cleaning build for',
                                      hint)
            return

        self.notify_part_progress('Cleaning build for', hint)

        if os.path.exists(self.code.build_basedir):
            shutil.rmtree(self.code.build_basedir)

        if os.path.exists(self.installdir):
            shutil.rmtree(self.installdir)

        self.code.clean_build()
        self.mark_cleaned('build')

    def migratable_fileset_for(self, step):
        plugin_fileset = self.code.snap_fileset()
        fileset = self._get_fileset(step).copy()
        includes = _get_includes(fileset)
        # If we're priming and we don't have an explicit set of files to prime
        # include the files from the stage step
        if step == 'prime' and (fileset == ['*'] or
                                len(includes) == 0):
            stage_fileset = self._get_fileset('stage').copy()
            fileset = _combine_filesets(stage_fileset, fileset)

        fileset.extend(plugin_fileset)

        return _migratable_filesets(fileset, self.code.installdir)

    def _get_fileset(self, option, default=None):
        if default is None:
            default = ['*']

        fileset = getattr(self.code.options, option, default)
        return fileset if fileset else default

    def _organize(self):
        fileset = self._get_fileset('organize', {})

        _organize_filesets(fileset.copy(), self.code.installdir)

    def stage(self, force=False):
        self.makedirs()
        self.notify_part_progress('Staging')
        self._organize()
        snap_files, snap_dirs = self.migratable_fileset_for('stage')

        def fixup_func(file_path):
            if os.path.islink(file_path):
                return
            if not file_path.endswith('.pc'):
                return
            repo.fix_pkg_config(self.stagedir, file_path, self.code.installdir)

        _migrate_files(snap_files, snap_dirs, self.code.installdir,
                       self.stagedir, fixup_func=fixup_func)
        # TODO once `snappy try` is in place we will need to copy
        # dependencies here too

        self.mark_stage_done(snap_files, snap_dirs)

    def mark_stage_done(self, snap_files, snap_dirs):
        self.mark_done('stage', states.StageState(
            snap_files, snap_dirs, self._part_properties,
            self._project_options))

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
        snap_files, snap_dirs = self.migratable_fileset_for('prime')
        _migrate_files(snap_files, snap_dirs, self.stagedir, self.snapdir)

        dependencies = _find_dependencies(self.snapdir, snap_files)

        # Split the necessary dependencies into their corresponding location.
        # We'll both migrate and track the system dependencies, but we'll only
        # track the part and staged dependencies, since they should have
        # already been primed by other means, and migrating them again could
        # potentially override the `stage` or `snap` filtering.
        (in_part, staged, primed, system) = _split_dependencies(
            dependencies, self.installdir, self.stagedir, self.snapdir)

        part_dependency_paths = {os.path.dirname(d) for d in in_part}
        staged_dependency_paths = {os.path.dirname(d) for d in staged}

        dependency_paths = part_dependency_paths | staged_dependency_paths

        if not self._build_attributes.no_system_libraries():
            system_dependency_paths = {os.path.dirname(d) for d in system}
            dependency_paths.update(system_dependency_paths)

            if system:
                # Lots of dependencies are linked with a symlink, so we need to
                # make sure we follow those symlinks when we migrate the
                # dependencies.
                _migrate_files(system, system_dependency_paths, '/',
                               self.snapdir, follow_symlinks=True)

        self.mark_prime_done(snap_files, snap_dirs, dependency_paths)

    def mark_prime_done(self, snap_files, snap_dirs, dependency_paths):
        self.mark_done('prime', states.PrimeState(
            snap_files, snap_dirs, dependency_paths, self._part_properties,
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


def _split_dependencies(dependencies, installdir, stagedir, snapdir):
    """Split dependencies into their corresponding location.

    Return a tuple of sets for each location.
    """

    part_dependencies = set()
    staged_dependencies = set()
    primed_dependencies = set()
    system_dependencies = set()

    for file_path in dependencies:
        if file_path.startswith(installdir):
            part_dependencies.add(os.path.relpath(file_path, installdir))
        elif file_path.startswith(stagedir):
            staged_dependencies.add(os.path.relpath(file_path, stagedir))
        elif file_path.startswith(snapdir):
            primed_dependencies.add(os.path.relpath(file_path, snapdir))
        else:
            file_path = file_path.lstrip('/')

            # This was a dependency that was resolved to be on the system.
            # However, it's possible that this library is actually included in
            # the snap and we just missed it because it's in a non-standard
            # path. Let's make sure it isn't already in the part, stage dir, or
            # prime dir. If so, add it to that set.
            if os.path.exists(os.path.join(installdir, file_path)):
                part_dependencies.add(file_path)
            elif os.path.exists(os.path.join(stagedir, file_path)):
                staged_dependencies.add(file_path)
            elif os.path.exists(os.path.join(snapdir, file_path)):
                primed_dependencies.add(file_path)
            else:
                system_dependencies.add(file_path)

    return (part_dependencies, staged_dependencies, primed_dependencies,
            system_dependencies)


def _expand_part_properties(part_properties, part_schema):
    """Returns properties with all part schema properties included.

    Any schema properties not set will contain their default value as defined
    in the schema itself.
    """

    # First make a deep copy of the part schema. It contains nested mutables,
    # and we'd rather not change them.
    part_schema = copy.deepcopy(part_schema)

    # Come up with a dictionary of part schema properties and their default
    # values as defined in the schema.
    properties = {}
    for schema_property, subschema in part_schema.items():
        properties[schema_property] = subschema.get('default')

    # Now expand (overwriting if necessary) the default schema properties with
    # the ones from the actual part.
    properties.update(part_properties)

    return properties


def _merged_part_and_plugin_schemas(part_schema, definitions_schema,
                                    plugin_schema):
    plugin_schema = plugin_schema.copy()
    if 'properties' not in plugin_schema:
        plugin_schema['properties'] = {}

    if 'definitions' not in plugin_schema:
        plugin_schema['definitions'] = {}

    # The part schema takes precedence over the plugin's schema.
    plugin_schema['properties'].update(part_schema)
    plugin_schema['definitions'].update(definitions_schema)

    return plugin_schema


def _validate_pull_and_build_properties(plugin_name, plugin, part_schema,
                                        definitions_schema):
    merged_schema = _merged_part_and_plugin_schemas(
        part_schema, definitions_schema, plugin.schema())
    merged_properties = merged_schema['properties']

    # First, validate pull properties
    invalid_properties = _validate_step_properties(
        plugin.get_pull_properties(), merged_properties)

    if invalid_properties:
        raise ValueError(
            "Invalid pull properties specified by {!r} plugin: {}".format(
                plugin_name, list(invalid_properties)))

    # Now, validate build properties
    invalid_properties = _validate_step_properties(
        plugin.get_build_properties(), merged_properties)

    if invalid_properties:
        raise ValueError(
            "Invalid build properties specified by {!r} plugin: {}".format(
                plugin_name, list(invalid_properties)))


def _validate_step_properties(step_properties, schema_properties):
    invalid_properties = set()
    for step_property in step_properties:
        if step_property not in schema_properties:
            invalid_properties.add(step_property)

    return invalid_properties


def _make_options(part_schema, definitions_schema, properties, plugin_schema):
    # Make copies as these dictionaries are tampered with
    part_schema = part_schema.copy()
    properties = properties.copy()

    plugin_schema = _merged_part_and_plugin_schemas(
        part_schema, definitions_schema, plugin_schema)

    # This is for backwards compatibility for when most of the
    # schema was overridable by the plugins.
    if 'required' in plugin_schema and not plugin_schema['required']:
            del plugin_schema['required']
    # With the same backwards compatibility in mind we need to remove
    # the source entry before validation. To those concerned, it has
    # already been validated.
    validated_properties = properties.copy()
    remove_set = [k for k in sources.get_source_defaults().keys()
                  if k in validated_properties]
    for key in remove_set:
        del validated_properties[key]

    jsonschema.validate(validated_properties, plugin_schema)

    options = _populate_options(properties, plugin_schema)

    return options


def _populate_options(properties, schema):
    class Options():
        pass

    options = Options()

    schema_properties = schema.get('properties', {})
    for key in schema_properties:
        attr_name = key.replace('-', '_')
        default_value = schema_properties[key].get('default')
        attr_value = properties.get(key, default_value)
        setattr(options, attr_name, attr_value)

    return options


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


def load_plugin(part_name, *, plugin_name, part_properties=None,
                project_options=None, part_schema=None,
                definitions_schema=None):
    if part_properties is None:
        part_properties = {}
    if part_schema is None:
        part_schema = {}
    if definitions_schema is None:
        definitions_schema = {}
    if project_options is None:
        project_options = snapcraft.ProjectOptions()
    logger.debug('Setting up part {!r} with plugin {!r} and '
                 'properties {!r}.'.format(part_name,
                                           plugin_name,
                                           part_properties))
    return PluginHandler(plugin_name=plugin_name,
                         part_name=part_name,
                         part_properties=part_properties,
                         project_options=project_options,
                         part_schema=part_schema,
                         definitions_schema=definitions_schema)


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
                   follow_symlinks=False, fixup_func=lambda *args: None):

    for directory in snap_dirs:
        src = os.path.join(srcdir, directory)
        dst = os.path.join(dstdir, directory)

        snapcraft.file_utils.create_similar_directory(src, dst)

    for snap_file in snap_files:
        src = os.path.join(srcdir, snap_file)
        dst = os.path.join(dstdir, snap_file)

        snapcraft.file_utils.create_similar_directory(os.path.dirname(src),
                                                      os.path.dirname(dst))

        if missing_ok and not os.path.exists(src):
            continue

        # If the file is already here and it's a symlink, leave it alone.
        if os.path.islink(dst):
            continue

        # Otherwise, remove and re-link it.
        if os.path.exists(dst):
            os.remove(dst)

        if src.endswith('.pc'):
            shutil.copy2(src, dst, follow_symlinks=follow_symlinks)
        else:
            file_utils.link_or_copy(src, dst, follow_symlinks=follow_symlinks)

        fixup_func(dst)


def _organize_filesets(fileset, base_dir):
    for key in sorted(fileset, key=lambda x: ['*' in x, x]):
        src = os.path.join(base_dir, key)
        dst = os.path.join(base_dir, fileset[key])

        sources = iglob(src, recursive=True)

        for src in sources:
            if os.path.isdir(src) and '*' not in key:
                file_utils.link_or_copy_tree(src, dst)
                # TODO create alternate organization location to avoid
                # deletions.
                shutil.rmtree(src)
            elif os.path.isfile(dst):
                raise EnvironmentError(
                    'Trying to organize file {key!r} to {dst!r}, '
                    'but {dst!r} already exists'.format(
                        key=key, dst=os.path.relpath(dst, base_dir)))
            else:
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                shutil.move(src, dst)


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


def _find_dependencies(root, part_files):
    ms = magic.open(magic.NONE)
    if ms.load() != 0:
        raise RuntimeError('Cannot load magic header detection')

    elf_files = set()

    fs_encoding = sys.getfilesystemencoding()

    for part_file in part_files:
        # Filter out object (*.o) files-- we only care about binaries.
        if part_file.endswith('.o'):
            continue

        # No need to crawl links-- the original should be here, too.
        path = os.path.join(root, part_file)
        if os.path.islink(path):
            logger.debug('Skipped link {!r} while finding dependencies'.format(
                path))
            continue

        path = path.encode(fs_encoding, errors='surrogateescape')
        # Finally, make sure this is actually an ELF before queueing it up
        # for an ldd call.
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
            pattern = os.path.join(directory, include)
            matches = iglob(pattern, recursive=True)
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
        pattern = os.path.join(directory, exclude)
        matches = iglob(pattern, recursive=True)
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


def _file_collides(file_this, file_other):
    if not file_this.endswith('.pc'):
        return not filecmp.cmp(file_this, file_other, shallow=False)

    pc_file_1 = open(file_this)
    pc_file_2 = open(file_other)

    try:
        for lines in zip(pc_file_1, pc_file_2):
            for line in zip(lines[0].split('\n'), lines[1].split('\n')):
                if line[0].startswith('prefix='):
                    continue
                if line[0] != line[1]:
                    return True
    except Exception as e:
        raise e from e
    finally:
        pc_file_1.close()
        pc_file_2.close()
    return False


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
                if _file_collides(this, other):
                    conflict_files.append(f)

            if conflict_files:
                raise SnapcraftPartConflictError(
                    other_part_name=other_part_name,
                    part_name=part.name,
                    conflict_files=conflict_files)

        # And add our files to the list
        parts_files[part.name] = {'files': part_files,
                                  'installdir': part.installdir}


def _get_includes(fileset):
    return [x for x in fileset if x[0] != '-']


def _get_excludes(fileset):
    return [x[1:] for x in fileset if x[0] == '-']


def _combine_filesets(starting_fileset, modifying_fileset):
    """
    Combine filesets if modifying_fileset is an explicit or implicit
    wildcard.
    """

    starting_excludes = set(_get_excludes(starting_fileset))
    modifying_includes = set(_get_includes(modifying_fileset))

    contradicting_fileset = set.intersection(starting_excludes,
                                             modifying_includes)

    if contradicting_fileset:
        raise PrimeFileConflictError(fileset=contradicting_fileset)

    to_combine = False
    # combine if starting_fileset has a wildcard
    # XXX: should this only be a single wildcard and possibly excludes?
    if '*' in modifying_fileset:
        to_combine = True
        modifying_fileset.remove('*')

    # combine if modifying_fileset is only excludes
    if set([x[0] for x in modifying_fileset]) == set('-'):
        to_combine = True

    if to_combine:
        return list(set(starting_fileset + modifying_fileset))
    else:
        return modifying_fileset
