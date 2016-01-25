# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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
import logging
import os
import shutil
import sys

import jsonschema
import yaml

import snapcraft
from snapcraft import (
    common,
    repo,
)

_SNAPCRAFT_STAGE = '$SNAPCRAFT_STAGE'

logger = logging.getLogger(__name__)


def _local_plugindir():
    return os.path.abspath(os.path.join('parts', 'plugins'))


class PluginError(Exception):
    pass


class PluginHandler:

    @property
    def name(self):
        return self._name

    @property
    def installdir(self):
        return self.code.installdir

    def __init__(self, plugin_name, part_name, properties):
        self.valid = False
        self.code = None
        self.config = {}
        self._name = part_name
        self.deps = []

        self.stagedir = os.path.join(os.getcwd(), 'stage')
        self.snapdir = os.path.join(os.getcwd(), 'snap')

        parts_dir = common.get_partsdir()
        self.ubuntudir = os.path.join(parts_dir, part_name, 'ubuntu')
        self.statedir = os.path.join(parts_dir, part_name, 'state')

        try:
            self._load_code(plugin_name, properties)
        except jsonschema.ValidationError as e:
            raise PluginError('properties failed to load for {}: {}'.format(
                part_name, e.message))

    def _load_code(self, plugin_name, properties):
        module_name = plugin_name.replace('-', '_')
        module = None

        with contextlib.suppress(ImportError):
            module = _load_local('x-{}'.format(plugin_name))
            logger.info('Loaded local plugin for %s', plugin_name)

        if not module:
            with contextlib.suppress(ImportError):
                module = importlib.import_module(
                    'snapcraft.plugins.{}'.format(module_name))

        if not module:
            logger.info('Searching for local plugin for %s', plugin_name)
            with contextlib.suppress(ImportError):
                module = _load_local(module_name)
            if not module:
                raise PluginError('unknown plugin: {}'.format(plugin_name))

        plugin = _get_plugin(module)
        options = _make_options(properties, plugin.schema())
        self.code = plugin(self.name, options)
        if common.host_machine != common.target_machine:
            logger.debug(
                'Setting {!r} as the compilation target for {!r}'.format(
                    common.target_machine, plugin_name))
            self.code.set_target_machine(common.target_machine)

    def makedirs(self):
        dirs = [
            self.code.sourcedir, self.code.builddir, self.code.installdir,
            self.stagedir, self.snapdir, self.ubuntudir, self.statedir
        ]
        for d in dirs:
            os.makedirs(d, exist_ok=True)

    def notify_stage(self, stage, hint=''):
        logger.info('%s %s %s', stage, self.name, hint)

    def last_step(self):
        for step in reversed(common.COMMAND_ORDER):
            if os.path.exists(self._step_state_file(step)):
                return step

        return None

    def is_dirty(self, step):
        last_step = self.last_step()
        if last_step:
            return (common.COMMAND_ORDER.index(step) >
                    common.COMMAND_ORDER.index(last_step))

        return True

    def should_step_run(self, step, force=False):
        return force or self.is_dirty(step)

    def mark_done(self, step):
        index = common.COMMAND_ORDER.index(step)

        # Currently only creating the step file. TODO: This will be a YAML file
        # holding step-specific state information.
        open(self._step_state_file(step), 'w').close()

        # We know we've only just completed this step, so make sure any later
        # steps don't have a saved state.
        if index+1 != len(common.COMMAND_ORDER):
            for command in common.COMMAND_ORDER[index+1:]:
                state_file = self._step_state_file(command)
                if os.path.exists(state_file):
                    os.remove(state_file)

    def _step_state_file(self, step):
        return os.path.join(self.statedir, step)

    def _setup_stage_packages(self):
        ubuntu = repo.Ubuntu(
            self.ubuntudir, sources=self.code.PLUGIN_STAGE_SOURCES)
        ubuntu.get(self.code.stage_packages)
        ubuntu.unpack(self.installdir)

        snap_files, snap_dirs = self.migratable_fileset_for('stage')
        _migrate_files(snap_files, snap_dirs, self.code.installdir,
                       self.stagedir, missing_ok=True)

    def pull(self, force=False):
        if not self.should_step_run('pull', force):
            self.notify_stage('Skipping pull', ' (already ran)')
            return
        self.makedirs()
        self.notify_stage('Pulling')
        if self.code.stage_packages:
            self._setup_stage_packages()
        self.code.pull()
        self.mark_done('pull')

    def build(self, force=False):
        if not self.should_step_run('build', force):
            self.notify_stage('Skipping build', ' (already ran)')
            return
        self.makedirs()
        self.notify_stage('Building')
        self.code.build()
        self.mark_done('build')

    def migratable_fileset_for(self, stage):
        plugin_fileset = self.code.snap_fileset()
        fileset = getattr(self.code.options, stage, ['*']) or ['*']
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
        if not self.should_step_run('stage', force):
            self.notify_stage('Skipping stage', ' (already ran)')
            return
        self.makedirs()

        self.notify_stage('Staging')
        self._organize()
        snap_files, snap_dirs = self.migratable_fileset_for('stage')
        _migrate_files(snap_files, snap_dirs, self.code.installdir,
                       self.stagedir)

        self.mark_done('stage')

    def strip(self, force=False):
        if not self.should_step_run('strip', force):
            self.notify_stage('Skipping strip', ' (already ran)')
            return
        self.makedirs()

        self.notify_stage('Stripping')
        snap_files, snap_dirs = self.migratable_fileset_for('snap')
        _migrate_files(snap_files, snap_dirs, self.stagedir, self.snapdir)

        self.mark_done('strip')

    def env(self, root):
        return self.code.env(root)

    def clean(self):
        logger.info('Cleaning up for part "{}"'.format(self.name))
        if os.path.exists(self.code.partdir):
            shutil.rmtree(self.code.partdir)


def _make_options(properties, schema):
    jsonschema.validate(properties, schema)

    class Options():
        pass
    options = Options()

    # Look at the system level props
    _populate_options(options, properties,
                      _system_schema_part_props())

    # Look at the plugin level props
    _populate_options(options, properties, schema)

    return options


def _system_schema_part_props():
    schema_file = os.path.abspath(os.path.join(common.get_schemadir(),
                                               'snapcraft.yaml'))

    try:
        with open(schema_file) as fp:
            schema = yaml.load(fp)
    except FileNotFoundError:
        raise FileNotFoundError(
            'snapcraft validation file is missing from installation path')

    props = {'properties': {}}
    partpattern = schema['properties']['parts']['patternProperties']
    for pattern in partpattern:
        props['properties'].update(partpattern[pattern]['properties'])

    return props


def _populate_options(options, properties, schema):
    schema_properties = schema.get('properties', {})
    for key in schema_properties:
        attr_name = key.replace('-', '_')
        default_value = schema_properties[key].get('default')
        attr_value = _expand_env(properties.get(key, default_value))
        setattr(options, attr_name, attr_value)


def _expand_env(attr):
    if isinstance(attr, str) and _SNAPCRAFT_STAGE in attr:
        return attr.replace(_SNAPCRAFT_STAGE, common.get_stagedir())
    elif isinstance(attr, list) or isinstance(attr, tuple):
        return [_expand_env(i) for i in attr]
    elif isinstance(attr, dict):
        return {k: _expand_env(attr[k]) for k in attr}

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


def _load_local(module_name):
    sys.path = [_local_plugindir()] + sys.path
    module = importlib.import_module(module_name)
    sys.path.pop(0)

    return module


def load_plugin(part_name, plugin_name, properties=None):
    if properties is None:
        properties = {}
    return PluginHandler(plugin_name, part_name, properties)


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

    return snap_files, snap_dirs


def _migrate_files(snap_files, snap_dirs, srcdir, dstdir, missing_ok=False):
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

        os.link(src, dst, follow_symlinks=False)


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
