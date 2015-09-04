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

import yaml

import snapcraft
from snapcraft import common
from snapcraft import repo


logger = logging.getLogger(__name__)


def is_local_plugin(name):
    return name.startswith("x-")


def plugindir(name):
    if is_local_plugin(name):
        return os.path.abspath(os.path.join('parts', 'plugins'))
    else:
        return common.get_plugindir()


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

        parts_dir = os.path.join(os.getcwd(), 'parts')
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
        configPath = os.path.join(plugindir(name), name + ".yaml")
        if not os.path.exists(configPath):
            raise PluginError('Unknown plugin: {}'.format(name))
        with open(configPath, 'r') as fp:
            self.config = yaml.load(fp) or {}

    def _make_options(self, name, properties):
        class Options():
            pass
        options = Options()

        for opt in self.config.get('options', []):
            attrname = opt.replace('-', '_')
            opt_parameters = self.config['options'][opt] or {}
            if opt in properties:
                setattr(options, attrname, properties[opt])
            else:
                if opt_parameters.get('required', False):
                    raise PluginError('Required field {} missing on part {}'.format(opt, name))
                setattr(options, attrname, None)

        return options

    def _load_code(self, name, part_name, properties):
        options = self._make_options(name, properties)
        moduleName = self.config.get('module', name)

        # Load code from local plugin dir if it is there
        if is_local_plugin(name):
            sys.path = [plugindir(name)] + sys.path
        else:
            moduleName = 'snapcraft.plugins.' + moduleName

        module = importlib.import_module(moduleName)
        if is_local_plugin(name):
            sys.path.pop(0)

        for propName in dir(module):
            prop = getattr(module, propName)
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

    def pull(self, force=False):
        if not self.should_stage_run('pull', force):
            return True
        self.makedirs()

        run_setup_stage_packages = self.code and hasattr(self.code, 'setup_stage_packages')
        run_pull = self.code and hasattr(self.code, 'pull')

        if run_setup_stage_packages or run_pull:
            self.notify_stage("Pulling")

        if run_setup_stage_packages:
            try:
                self.code.setup_stage_packages()
            except repo.PackageNotFoundError as e:
                logger.error(e.message)
                return False

        if run_pull:
            if not getattr(self.code, 'pull')():
                return False

        self.mark_done('pull')
        return True

    def build(self, force=False):
        if not self.should_stage_run('build', force):
            return True
        self.makedirs()
        if self.code and hasattr(self.code, 'build'):
            self.notify_stage("Building")
            if not getattr(self.code, 'build')():
                return False

        self.mark_done('build')
        return True

    def stage(self, force=False):
        if not self.should_stage_run('stage', force):
            return True
        self.makedirs()
        if not self.code:
            return True

        self.notify_stage("Staging")
        common.run(['cp', '-arT', self.installdir, self.stagedir])
        self.mark_done('stage')
        return True

    def snap(self, force=False):
        if not self.should_stage_run('snap', force):
            return True
        self.makedirs()

        if self.code and hasattr(self.code, 'snap_files'):
            self.notify_stage("Snapping")

            includes, excludes = getattr(self.code, 'snap_files')()
            snapDirs, snap_files = self.collect_snap_files(includes, excludes)

            if snapDirs:
                common.run(['mkdir', '-p'] + list(snapDirs), cwd=self.stagedir)
            if snap_files:
                common.run(['cp', '-a', '--parent'] + list(snap_files) + [self.snapdir], cwd=self.stagedir)

            self.mark_done('snap')
        return True

    def collect_snap_files(self, includes, excludes):
        # validate
        _validate_relative_paths(includes + excludes)

        source_files = _generate_source_set(self.installdir)
        include_files = _generate_include_set(self.stagedir, includes)
        exclude_files, exclude_dirs = _generate_exclude_set(self.stagedir, excludes)

        # And chop files, including whole trees if any dirs are mentioned
        snap_files = (include_files & source_files) - exclude_files
        for exclude_dir in exclude_dirs:
            snap_files = set([x for x in snap_files if not x.startswith(exclude_dir + '/')])

        # Separate dirs from files
        snap_dirs = set([x for x in snap_files if os.path.isdir(os.path.join(self.stagedir, x)) and not os.path.islink(os.path.join(self.stagedir, x))])
        snap_files = snap_files - snap_dirs

        return snap_dirs, snap_files

    def env(self, root):
        if self.code and hasattr(self.code, 'env'):
            return getattr(self.code, 'env')(root)
        return []


def load_plugin(part_name, plugin_name, properties={}, load_code=True):
    part = PluginHandler(plugin_name, part_name, properties, load_code=load_code)
    if not part.is_valid():
        logger.error('Could not load part %s', plugin_name)
        sys.exit(1)
    return part


def _generate_source_set(installdir):
    source_files = set()
    for root, dirs, files in os.walk(installdir):
        source_files |= set([os.path.join(root, d) for d in dirs])
        source_files |= set([os.path.join(root, f) for f in files])
    source_files = set([os.path.relpath(x, installdir) for x in source_files])

    return source_files


def _generate_include_set(stagedir, includes):
    include_files = set()
    for include in includes:
        matches = glob.glob(os.path.join(stagedir, include))
        include_files |= set(matches)

    include_dirs = [x for x in include_files if os.path.isdir(x)]
    include_files = set([os.path.relpath(x, stagedir) for x in include_files])

    # Expand includeFiles, so that an exclude like '*/*.so' will still match
    # files from an include like 'lib'
    for include_dir in include_dirs:
        for root, dirs, files in os.walk(include_dir):
            include_files |= set([os.path.relpath(os.path.join(root, d), stagedir) for d in dirs])
            include_files |= set([os.path.relpath(os.path.join(root, f), stagedir) for f in files])

    return include_files


def _generate_exclude_set(stagedir, excludes):
    exclude_files = set()

    for exclude in excludes:
        matches = glob.glob(os.path.join(stagedir, exclude))
        exclude_files |= set(matches)

    exclude_dirs = [os.path.relpath(x, stagedir) for x in exclude_files if os.path.isdir(x)]
    exclude_files = set([os.path.relpath(x, stagedir) for x in exclude_files])

    return exclude_files, exclude_dirs


def _validate_relative_paths(files):
    for d in files:
        if os.path.isabs(d):
            raise PluginError("path '{}' must be relative".format(d))
