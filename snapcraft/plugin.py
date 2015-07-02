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

import importlib
import os
import snapcraft
import snapcraft.common
import subprocess
import sys
import yaml


class Plugin:

    def __init__(self, name, partName, properties, optionsOverride=None, loadCode=True, loadConfig=True):
        self.valid = False
        self.code = None
        self.config = None
        self.partNames = []

        self.sourcedir = os.path.join(os.getcwd(), "parts", partName, "src")
        self.builddir = os.path.join(os.getcwd(), "parts", partName, "build")
        self.stagedir = os.path.join(os.getcwd(), "stage")
        self.snapdir = os.path.join(os.getcwd(), "snap")
        self.statefile = os.path.join(os.getcwd(), "parts", partName, "state")

        if loadConfig:
            # FIXME: use env var for this
            pluginDir = os.path.abspath(os.path.join(__file__, '..', '..', 'plugins'))
            configPath = os.path.join(pluginDir, name + ".yaml")
            if not os.path.exists(configPath):
                snapcraft.common.log("Missing config for part %s" % (name), file=sys.stderr)
                return
            self.config = yaml.load(open(configPath, 'r')) or {}

            if loadCode:
                class Options():
                    pass
                options = Options()

                if self.config:
                    for opt in self.config.get('options', []):
                        if opt in properties:
                            setattr(options, opt, properties[opt])
                        else:
                            if self.config['options'][opt].get('required', False):
                                snapcraft.common.log("Required field %s missing on part %s" % (opt, name), file=sys.stderr)
                                return
                            setattr(options, opt, None)
                if optionsOverride:
                    options = optionsOverride

                moduleName = self.config.get('module', name)
                module = importlib.import_module("snapcraft.plugins." + moduleName)
                for propName in dir(module):
                    prop = getattr(module, propName)
                    if issubclass(prop, snapcraft.BasePlugin):
                        self.code = prop(partName, options)
                        break

        self.partNames.append(partName)
        self.valid = True

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
            os.makedirs(self.stagedir)
        except FileExistsError:
            pass
        try:
            os.makedirs(self.snapdir)
        except FileExistsError:
            pass

    def isValid(self):
        return self.valid

    def names(self):
        return self.partNames

    def notifyStage(self, stage, hint=''):
        snapcraft.common.log(stage + " " + self.partNames[0] + hint)

    def isDirty(self, stage):
        try:
            with open(self.statefile, 'r') as f:
                lastStep = f.read()
                return snapcraft.common.commandOrder.index(stage) > snapcraft.common.commandOrder.index(lastStep)
        except Exception:
            return True

    def shouldStageRun(self, stage, force):
        if not force and not self.isDirty(stage):
            self.notifyStage('Skipping ' + stage, ' (already ran)')
            return False
        return True

    def markDone(self, stage):
        with open(self.statefile, 'w+') as f:
            f.write(stage)

    def pull(self, force=False):
        if not self.shouldStageRun('pull', force):
            return True
        self.makedirs()
        if self.code and hasattr(self.code, 'pull'):
            self.notifyStage("Pulling")
            if not getattr(self.code, 'pull')():
                return False
            self.markDone('pull')
        return True

    def build(self, force=False):
        if not self.shouldStageRun('build', force):
            return True
        self.makedirs()
        subprocess.call(['cp', '-Trf', self.sourcedir, self.builddir])
        if self.code and hasattr(self.code, 'build'):
            self.notifyStage("Building")
            if not getattr(self.code, 'build')():
                return False
            self.markDone('build')
        return True

    def test(self, force=False):
        if not self.shouldStageRun('test', force):
            return True
        self.makedirs()
        if self.code and hasattr(self.code, 'test'):
            self.notifyStage("Testing")
            if not getattr(self.code, 'test')():
                return False
            self.markDone('test')
        return True

    def stage(self, force=False):
        if not self.shouldStageRun('stage', force):
            return True
        self.makedirs()
        if self.code and hasattr(self.code, 'stage'):
            self.notifyStage("Staging")
            if not getattr(self.code, 'stage')():
                return False
            self.markDone('stage')
        return True

    def snap(self, force=False):
        if not self.shouldStageRun('snap', force):
            return True
        self.makedirs()
        if self.code and hasattr(self.code, 'snap'):
            self.notifyStage("Snapping")
            if not getattr(self.code, 'snap')():
                return False
            self.markDone('snap')
        return True

    def env(self):
        if self.code and hasattr(self.code, 'env'):
            return getattr(self.code, 'env')()
        return []


def loadPlugin(partName, pluginName, properties={}, loadCode=True):
    part = Plugin(pluginName, partName, properties, loadCode=loadCode)
    if not part.isValid():
        snapcraft.common.log("Could not load part %s" % pluginName, file=sys.stderr)
        sys.exit(1)
    return part
