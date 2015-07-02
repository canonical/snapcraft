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

import snapcraft.common
import snapcraft.plugin
import sys
import yaml


class Config:

    def __init__(self):
        self.systemPackages = []
        self.allParts = []
        afterRequests = {}

        try:
            self.data = yaml.load(open("snapcraft.yaml", 'r'))
        except FileNotFoundError:
            snapcraft.common.log("Could not find snapcraft.yaml.  Are you sure you're in the right directory?\nTo start a new project, use 'snapcraft init'")
            sys.exit(1)
        self.systemPackages = self.data.get('systemPackages', [])

        for partName in self.data.get("parts", []):
            properties = self.data["parts"][partName]

            pluginName = properties.get("plugin", partName)
            if "plugin" in properties:
                del properties["plugin"]

            if "after" in properties:
                afterRequests[partName] = properties["after"]
                del properties["after"]

            self.loadPlugin(partName, pluginName, properties)

        # Grab all required dependencies, if not already specified
        newParts = self.allParts.copy()
        while newParts:
            part = newParts.pop(0)
            requires = part.config.get('requires', [])
            for requiredPart in requires:
                alreadyPresent = False
                for p in self.allParts:
                    if requiredPart in p.names():
                        alreadyPresent = True
                        break
                if not alreadyPresent:
                    newParts.append(self.loadPlugin(requiredPart, requiredPart, {}))

        # Now sort them
        partsToSort = self.allParts.copy()
        while partsToSort:
            part = partsToSort.pop(0)
            requires = part.config.get('requires', [])
            for requiredPart in requires:
                for i in range(len(self.allParts)):
                    if requiredPart in self.allParts[i].names():
                        self.allParts.insert(0, self.allParts.pop(i))
                        break
            afterNames = afterRequests.get(part.names()[0], [])
            for afterName in afterNames:
                for i in range(len(self.allParts)):
                    if afterName in self.allParts[i].names():
                        self.allParts.insert(0, self.allParts.pop(i))
                        break

    def loadPlugin(self, partName, pluginName, properties, loadCode=True):
        part = snapcraft.plugin.loadPlugin(partName, pluginName, properties, loadCode=loadCode)

        self.systemPackages += part.config.get('systemPackages', [])
        self.allParts.append(part)
        return part

    def env(self):
        env = []
        env.append("PATH=\"%s/bin:%s/usr/bin:$PATH\"" % (snapcraft.common.stagedir, snapcraft.common.stagedir))
        env.append("LD_LIBRARY_PATH=\"%s/lib:%s/usr/lib:$LD_LIBRARY_PATH\"" % (snapcraft.common.stagedir, snapcraft.common.stagedir))
        env.append("CFLAGS=\"-I%s/include $CFLAGS\"" % snapcraft.common.stagedir)
        env.append("LDFLAGS=\"-L%s/lib $LDFLAGS\"" % snapcraft.common.stagedir)

        for part in self.allParts:
            env += part.env()

        return env
