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
import sys
import shutil

import jsonschema
import yaml

import snapcraft
import snapcraft.yaml

from snapcraft import (
    common,
    meta,
    repo,
)


logger = logging.getLogger(__name__)


def _local_plugindir():
    return os.path.abspath(os.path.join('parts', 'plugins'))


def execute(step, part_names=None):
    # TODO: add a docstring once strip is implemented.
    config = snapcraft.yaml.load_config()
    repo.install_build_packages(config.build_tools)

    if part_names:
        config.validate_parts(part_names)
        parts = {p for p in config.all_parts if p.name in part_names}
    else:
        parts = config.all_parts
        part_names = config.part_names

    step_index = common.COMMAND_ORDER.index(step) + 1
    for step in common.COMMAND_ORDER[0:step_index]:
        if step == 'stage':
            _check_for_collisions(config.all_parts)
        for part in parts:
            prereqs = config.part_prereqs(part.name)
            if prereqs and not prereqs.issubset(part_names):
                raise RuntimeError(
                    'Requested {!r} of {!r} but there are unsatisfied '
                    'prerequisites: {!r}'.format(
                        step, part.name, ' '.join(prereqs)))
            elif prereqs:
                # prerequisites need to build all the way to the staging
                # step to be able to share the common assets that make them
                # a dependency.
                logger.info(
                    '{!r} has prerequisites that need to be staged: {}'.format(
                        part.name, ' '.join(prereqs)))
                execute('stage', prereqs)
            common.env = config.build_env_for_part(part)
            getattr(part, step)()

        if step == 'strip' and part_names == config.part_names:
            meta.create(config.data)


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

        parts_dir = common.get_partsdir()
        self.ubuntudir = os.path.join(parts_dir, part_name, 'ubuntu')
        self.stagedir = os.path.join(os.getcwd(), 'stage')
        self.snapdir = os.path.join(os.getcwd(), 'snap')
        self.statefile = os.path.join(parts_dir, part_name, 'state')

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

    def makedirs(self):
        dirs = [
            self.code.sourcedir, self.code.builddir, self.code.installdir,
            self.stagedir, self.snapdir, self.ubuntudir
        ]
        for d in dirs:
            os.makedirs(d, exist_ok=True)

    def notify_stage(self, stage, hint=''):
        logger.info('%s %s %s', stage, self.name, hint)

    def is_dirty(self, stage):
        try:
            with open(self.statefile, 'r') as f:
                lastStep = f.read()
                return (common.COMMAND_ORDER.index(stage) >
                        common.COMMAND_ORDER.index(lastStep))
        except Exception:
            return True

    def should_stage_run(self, stage, force):
        if not force and not self.is_dirty(stage):
            self.notify_stage('Skipping ' + stage, ' (already ran)')
            return False
        return True

    def mark_done(self, stage):
        with open(self.statefile, 'w+') as f:
            f.write(stage)

    def _setup_stage_packages(self):
        if self.code.stage_packages:
            ubuntu = repo.Ubuntu(
                self.ubuntudir, sources=self.code.PLUGIN_STAGE_SOURCES)
            ubuntu.get(self.code.stage_packages)
            ubuntu.unpack(self.code.installdir)

    def pull(self, force=False):
        if not self.should_stage_run('pull', force):
            return
        self.makedirs()
        self.notify_stage('Pulling')
        self._setup_stage_packages()
        self.code.pull()
        self.mark_done('pull')

    def build(self, force=False):
        if not self.should_stage_run('build', force):
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
        if not self.should_stage_run('stage', force):
            return
        self.makedirs()

        self.notify_stage('Staging')
        self._organize()
        snap_files, snap_dirs = self.migratable_fileset_for('stage')

        try:
            _migrate_files(snap_files, snap_dirs, self.code.installdir,
                           self.stagedir)
        except FileNotFoundError as e:
            logger.error('Could not find file %s defined in stage',
                         os.path.relpath(e.filename, os.path.curdir))
            return False

        self.mark_done('stage')

        return True

    def strip(self, force=False):
        if not self.should_stage_run('strip', force):
            return
        self.makedirs()

        self.notify_stage('Stripping')
        snap_files, snap_dirs = self.migratable_fileset_for('snap')

        try:
            _migrate_files(snap_files, snap_dirs, self.stagedir, self.snapdir)
        except FileNotFoundError as e:
                logger.error('Could not find file %s defined in snap',
                             os.path.relpath(e.filename, os.path.curdir))
                return False

        self.mark_done('strip')

        return True

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
        attr_value = properties.get(key, default_value)
        setattr(options, attr_name, attr_value)


def _get_plugin(module):
    for attr in vars(module).values():
        if isinstance(attr, type) and issubclass(attr, snapcraft.BasePlugin):
            return attr


def _load_local(module_name):
    sys.path = [_local_plugindir()] + sys.path
    module = importlib.import_module(module_name)
    sys.path.pop(0)

    return module


def load_plugin(part_name, plugin_name, properties={}):
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


def _migrate_files(snap_files, snap_dirs, srcdir, dstdir):
    for directory in snap_dirs:
        os.makedirs(os.path.join(dstdir, directory), exist_ok=True)

    for snap_file in snap_files:
        src = os.path.join(srcdir, snap_file)
        dst = os.path.join(dstdir, snap_file)
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        if not os.path.exists(dst) and not os.path.islink(dst):
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


def _check_for_collisions(parts):
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
