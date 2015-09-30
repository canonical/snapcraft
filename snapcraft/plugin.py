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

import glob
import importlib
import logging
import os
import sys
import shutil
import yaml

import snapcraft
from snapcraft import common
from snapcraft import repo


logger = logging.getLogger(__name__)


_BUILTIN_OPTIONS = {
    'filesets': {},
    'snap': [],
    'stage': [],
    'stage-packages': [],
    'organize': {}
}


def _local_plugindir():
    return os.path.abspath(os.path.join('parts', 'plugins'))


class PluginError(Exception):
    pass


class PluginHandler:

    def __init__(self, name, part_name, properties, load_code=True, load_config=True):
        self.valid = False
        self.code = None
        self.config = {}
        self.part_names = []
        self.deps = []
        self.plugin_name = name

        parts_dir = common.get_partsdir()
        self.partdir = os.path.join(parts_dir, part_name)
        self.sourcedir = os.path.join(parts_dir, part_name, 'src')
        self.builddir = os.path.join(parts_dir, part_name, 'build')
        self.ubuntudir = os.path.join(parts_dir, part_name, 'ubuntu')
        self.installdir = os.path.join(parts_dir, part_name, 'install')
        self.stagedir = os.path.join(os.getcwd(), 'stage')
        self.snapdir = os.path.join(os.getcwd(), 'snap')
        self.statefile = os.path.join(parts_dir, part_name, 'state')

        try:
            if load_config:
                self._load_config(name)
            if load_code:
                self._load_code(name, part_name, properties)
            # only set to valid if it loads without PluginError
            self.part_names.append(part_name)
            self.valid = True
        except PluginError as e:
            logger.error(str(e))
            return

    def _load_config(self, name):
        config_path = os.path.join(common.get_plugindir(), name + ".yaml")
        if not os.path.exists(config_path):
            config_path = os.path.join(_local_plugindir(), name + ".yaml")
        if not os.path.exists(config_path):
            raise PluginError('Unknown plugin: {}'.format(name))
        with open(config_path, 'r') as fp:
            self.config = yaml.load(fp) or {}

    def _make_options(self, name, properties):
        class Options():
            pass
        options = Options()

        plugin_options = self.config.get('options', {})
        # Let's append some mandatory options, but not .update() to not lose
        # original content
        for key in _BUILTIN_OPTIONS:
            if key not in plugin_options:
                plugin_options[key] = _BUILTIN_OPTIONS[key]

        for opt in plugin_options:
            attrname = opt.replace('-', '_')
            opt_parameters = plugin_options[opt] or {}
            if opt in properties:
                setattr(options, attrname, properties[opt])
            else:
                if opt_parameters.get('required', False):
                    raise PluginError('Required field {} missing on part {}'.format(opt, name))
                setattr(options, attrname, None)

        return options

    def _load_code(self, name, part_name, properties):
        options = self._make_options(name, properties)
        module_name = name.replace('-', '_')

        try:
            module = importlib.import_module('snapcraft.plugins.' + module_name)
        except ImportError:
            module = None

        if not module:
            logger.info('Searching for local plugin for %s', name)
            sys.path = [_local_plugindir()] + sys.path
            module = importlib.import_module(module_name)
            sys.path.pop(0)

        for prop_name in dir(module):
            prop = getattr(module, prop_name)
            if issubclass(prop, snapcraft.BasePlugin):
                self.code = prop(part_name, options)
                break

    def __str__(self):
        return self.part_names[0]

    def __repr__(self):
        return self.part_names[0]

    def makedirs(self):
        dirs = [
            self.sourcedir, self.builddir, self.installdir, self.stagedir,
            self.snapdir, self.ubuntudir
        ]
        for d in dirs:
            os.makedirs(d, exist_ok=True)

    def is_valid(self):
        return self.valid

    def names(self):
        return self.part_names

    def notify_stage(self, stage, hint=''):
        logger.info(stage + " " + self.part_names[0] + hint)

    def is_dirty(self, stage):
        try:
            with open(self.statefile, 'r') as f:
                lastStep = f.read()
                return common.COMMAND_ORDER.index(stage) > common.COMMAND_ORDER.index(lastStep)
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

    def pull(self, force=False, config={}):
        if not self.should_stage_run('pull', force):
            return True
        self.makedirs()

        run_setup_stage_packages = self.code and hasattr(self.code, 'setup_stage_packages')
        run_pull = self.code and hasattr(self.code, 'pull')

        if run_setup_stage_packages or run_pull:
            self.notify_stage("Pulling")

        if run_pull:
            if not getattr(self.code, 'pull')():
                return False

        if run_setup_stage_packages:
            try:
                self.code.setup_stage_packages()
            except repo.PackageNotFoundError as e:
                logger.error(e.message)
                return False

        self.mark_done('pull')
        return True

    def build(self, force=False, config={}):
        if not self.should_stage_run('build', force):
            return True
        self.makedirs()
        if self.code and hasattr(self.code, 'build'):
            self.notify_stage("Building")
            if not getattr(self.code, 'build')():
                return False

        self.mark_done('build')
        return True

    def _migratable_fileset_for(self, stage):
        plugin_fileset = self.code.snap_fileset()
        fileset = getattr(self.code.options, stage, []) or []
        fileset.extend(plugin_fileset)
        return migratable_filesets(fileset, self.installdir)

    def _organize(self):
        organize_fileset = getattr(self.code.options, 'organize', {}) or {}

        for key in organize_fileset:
            src = os.path.join(self.installdir, key)
            dst = os.path.join(self.installdir, organize_fileset[key])

            os.makedirs(os.path.dirname(dst), exist_ok=True)

            if os.path.exists(dst):
                logger.warning(
                    'Stepping over existing file for organization %r',
                    os.path.relpath(dst, self.installdir))
                if os.path.isdir(dst):
                    shutil.rmtree(dst)
                else:
                    os.remove(dst)
            shutil.move(src, dst)

    def stage(self, force=False, config={}):
        if not self.should_stage_run('stage', force):
            return True
        self.makedirs()
        if not self.code:
            return True

        self.notify_stage("Staging")

        if self.code and hasattr(self.code, 'stage'):
            if not getattr(self.code, 'stage')():
                return False

        self._organize()
        snap_files, snap_dirs = self._migratable_fileset_for('stage')

        try:
            _migrate_files(snap_files, snap_dirs, self.installdir, self.stagedir)
        except FileNotFoundError as e:
            logger.error('Could not find file %s defined in stage',
                         os.path.relpath(e.filename, os.path.curdir))
            return False

        self.mark_done('stage')

        return True

    def snap(self, force=False, config={}):
        if not self.should_stage_run('snap', force):
            return True
        self.makedirs()

        self.notify_stage("Snapping")
        snap_files, snap_dirs = self._migratable_fileset_for('snap')

        try:
            _migrate_files(snap_files, snap_dirs, self.stagedir, self.snapdir)
        except FileNotFoundError as e:
                logger.error('Could not find file %s defined in snap',
                             os.path.relpath(e.filename, os.path.curdir))
                return False

        if self.code and hasattr(self.code, 'snap'):
            if not getattr(self.code, 'snap')(config=config):
                return False

        self.mark_done('snap')

        return True

    def env(self, root):
        if self.code and hasattr(self.code, 'env'):
            return self.code.env(root)
        return []


def load_plugin(part_name, plugin_name, properties={}, load_code=True):
    part = PluginHandler(plugin_name, part_name, properties, load_code=load_code)
    if not part.is_valid():
        logger.error('Could not load part %s', plugin_name)
        sys.exit(1)
    return part


def migratable_filesets(fileset, srcdir):
    includes, excludes = _get_file_list(fileset)

    include_files = _generate_include_set(srcdir, includes)
    exclude_files, exclude_dirs = _generate_exclude_set(srcdir, excludes)

    # And chop files, including whole trees if any dirs are mentioned
    snap_files = include_files - exclude_files
    for exclude_dir in exclude_dirs:
        snap_files = set([x for x in snap_files if not x.startswith(exclude_dir + '/')])

    # Separate dirs from files
    snap_dirs = set([x for x in snap_files if os.path.isdir(os.path.join(srcdir, x)) and not os.path.islink(os.path.join(srcdir, x))])
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
            include_files |= set([os.path.relpath(os.path.join(root, d), directory) for d in dirs])
            include_files |= set([os.path.relpath(os.path.join(root, f), directory) for f in files])

    return include_files


def _generate_exclude_set(directory, excludes):
    exclude_files = set()

    for exclude in excludes:
        matches = glob.glob(os.path.join(directory, exclude))
        exclude_files |= set(matches)

    exclude_dirs = [os.path.relpath(x, directory) for x in exclude_files if os.path.isdir(x)]
    exclude_files = set([os.path.relpath(x, directory) for x in exclude_files])

    return exclude_files, exclude_dirs


def _validate_relative_paths(files):
    for d in files:
        if os.path.isabs(d):
            raise PluginError("path '{}' must be relative".format(d))
