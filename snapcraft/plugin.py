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
import snapcraft
import snapcraft.common
import sys
import yaml


logger = logging.getLogger(__name__)


class Plugin:

    def __init__(self, name, part_name, properties, options_override=None, load_code=True, load_config=True):
        self.valid = False
        self.code = None
        self.config = None
        self.part_names = []
        self.deps = []
        self.plugin_name = name
        self.is_local_plugin = False

        self.sourcedir = os.path.join(os.getcwd(), "parts", part_name, "src")
        self.builddir = os.path.join(os.getcwd(), "parts", part_name, "build")
        self.installdir = os.path.join(os.getcwd(), "parts", part_name, "install")
        self.stagedir = os.path.join(os.getcwd(), "stage")
        self.snapdir = os.path.join(os.getcwd(), "snap")
        self.statefile = os.path.join(os.getcwd(), "parts", part_name, "state")

        if load_config:
            # First look in local path
            localPluginDir = os.path.abspath(os.path.join('parts', 'plugins'))
            configPath = os.path.join(localPluginDir, name + ".yaml")
            if os.path.exists(configPath):
                self.is_local_plugin = True
            else:
                # OK, now look at snapcraft's plugins
                configPath = os.path.join(snapcraft.common.plugindir, name + ".yaml")
                if not os.path.exists(configPath):
                    logger.error('Unknown plugin %s' % name)
                    return
            with open(configPath, 'r') as fp:
                self.config = yaml.load(fp) or {}

            if load_code:
                class Options():
                    pass
                options = Options()

                if self.config:
                    for opt in self.config.get('options', []):
                        if opt in properties:
                            setattr(options, opt, properties[opt])
                        else:
                            if self.config['options'][opt].get('required', False):
                                logger.error('Required field %s missing on part %s' % (opt, name))
                                return
                            setattr(options, opt, None)
                if options_override:
                    options = options_override

                moduleName = self.config.get('module', name)

                # Load code from local plugin dir if it is there
                if self.is_local_plugin:
                    sys.path = [localPluginDir] + sys.path
                else:
                    moduleName = 'snapcraft.plugins.' + moduleName

                module = importlib.import_module(moduleName)

                if self.is_local_plugin:
                    sys.path.pop(0)

                for propName in dir(module):
                    prop = getattr(module, propName)
                    if issubclass(prop, snapcraft.BasePlugin):
                        self.code = prop(part_name, options)
                        break

        self.part_names.append(part_name)
        self.valid = True

    def __str__(self):
        return self.part_names[0]

    def __repr__(self):
        return self.part_names[0]

    def makedirs(self):
        try:
            os.makedirs(self.sourcedir)
        except FileExistsError:
            pass
        try:
            os.makedirs(self.builddir)
        except FileExistsError:
            pass
        try:
            os.makedirs(self.installdir)
        except FileExistsError:
            pass
        try:
            os.makedirs(self.stagedir)
        except FileExistsError:
            pass
        try:
            os.makedirs(self.snapdir)
        except FileExistsError:
            pass

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
                return snapcraft.common.commandOrder.index(stage) > snapcraft.common.commandOrder.index(lastStep)
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
        if self.code and hasattr(self.code, 'pull'):
            self.notify_stage("Pulling")
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
        snapcraft.common.run(['cp', '-arT', self.installdir, self.stagedir])
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
                snapcraft.common.run(['mkdir', '-p'] + list(snapDirs), cwd=self.stagedir)
            if snap_files:
                snapcraft.common.run(['cp', '-a', '--parent'] + list(snap_files) + [self.snapdir], cwd=self.stagedir)

            self.mark_done('snap')
        return True

    def collect_snap_files(self, includes, excludes):
        sourceFiles = set()
        for root, dirs, files in os.walk(self.installdir):
            sourceFiles |= set([os.path.join(root, d) for d in dirs])
            sourceFiles |= set([os.path.join(root, f) for f in files])
        sourceFiles = set([os.path.relpath(x, self.installdir) for x in sourceFiles])

        includeFiles = set()
        for include in includes:
            matches = glob.glob(os.path.join(self.stagedir, include))
            includeFiles |= set(matches)
        includeDirs = [x for x in includeFiles if os.path.isdir(x)]
        includeFiles = set([os.path.relpath(x, self.stagedir) for x in includeFiles])

        # Expand includeFiles, so that an exclude like '*/*.so' will still match
        # files from an include like 'lib'
        for includeDir in includeDirs:
            for root, dirs, files in os.walk(includeDir):
                includeFiles |= set([os.path.relpath(os.path.join(root, d), self.stagedir) for d in dirs])
                includeFiles |= set([os.path.relpath(os.path.join(root, f), self.stagedir) for f in files])

        # Grab exclude list
        excludeFiles = set()
        for exclude in excludes:
            matches = glob.glob(os.path.join(self.stagedir, exclude))
            excludeFiles |= set(matches)
        excludeDirs = [os.path.relpath(x, self.stagedir) for x in excludeFiles if os.path.isdir(x)]
        excludeFiles = set([os.path.relpath(x, self.stagedir) for x in excludeFiles])

        # And chop files, including whole trees if any dirs are mentioned
        snap_files = (includeFiles & sourceFiles) - excludeFiles
        for excludeDir in excludeDirs:
            snap_files = set([x for x in snap_files if not x.startswith(excludeDir + '/')])

        # Separate dirs from files
        snapDirs = set([x for x in snap_files if os.path.isdir(os.path.join(self.stagedir, x))])
        snap_files = snap_files - snapDirs

        return snapDirs, snap_files

    def env(self, root):
        if self.code and hasattr(self.code, 'env'):
            return getattr(self.code, 'env')(root)
        return []


def load_plugin(part_name, plugin_name, properties={}, load_code=True):
    part = Plugin(plugin_name, part_name, properties, load_code=load_code)
    if not part.is_valid():
        logger.error('Could not load part %s' % plugin_name)
        sys.exit(1)
    return part
