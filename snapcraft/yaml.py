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
        self.all_parts = []
        afterRequests = {}

        try:
            with open("snapcraft.yaml", 'r') as fp:
                self.data = yaml.load(fp)
        except FileNotFoundError:
            snapcraft.common.log("Could not find snapcraft.yaml.  Are you sure you're in the right directory?\nTo start a new project, use 'snapcraft init'")
            sys.exit(1)
        self.systemPackages = self.data.get('systemPackages', [])

        for part_name in self.data.get("parts", []):
            properties = self.data["parts"][part_name] or {}

            plugin_name = properties.get("plugin", part_name)
            if "plugin" in properties:
                del properties["plugin"]

            if "after" in properties:
                afterRequests[part_name] = properties["after"]
                del properties["after"]

            # TODO: support 'filter' or 'blacklist' field to filter what gets put in snap/

            self.load_plugin(part_name, plugin_name, properties)

        # Grab all required dependencies, if not already specified
        newParts = self.all_parts.copy()
        while newParts:
            part = newParts.pop(0)
            requires = part.config.get('requires', [])
            for requiredPart in requires:
                alreadyPresent = False
                for p in self.all_parts:
                    if requiredPart in p.names():
                        alreadyPresent = True
                        break
                if not alreadyPresent:
                    newParts.append(self.load_plugin(requiredPart, requiredPart, {}))

        # Gather lists of dependencies
        for part in self.all_parts:
            depNames = part.config.get('requires', []) + afterRequests.get(part.names()[0], [])
            for dep in depNames:
                foundIt = False
                for i in range(len(self.all_parts)):
                    if dep in self.all_parts[i].names():
                        part.deps.append(self.all_parts[i])
                        foundIt = True
                        break
                if not foundIt:
                    snapcraft.common.log("Could not find part name %s" % dep)
                    sys.exit(1)

        # Now sort them (this is super inefficient, but easy-ish to follow)
        sortedParts = []
        while self.all_parts:
            topPart = None
            for part in self.all_parts:
                mentioned = False
                for other in self.all_parts:
                    if part in other.deps:
                        mentioned = True
                        break
                if not mentioned:
                    topPart = part
                    break
            if not topPart:
                snapcraft.common.log("Circular dependency chain!")
                sys.exit(1)
            sortedParts = [topPart] + sortedParts
            self.all_parts.remove(topPart)
        self.all_parts = sortedParts

    def load_plugin(self, part_name, plugin_name, properties, load_code=True):
        part = snapcraft.plugin.load_plugin(part_name, plugin_name, properties, load_code=load_code)

        self.systemPackages += part.config.get('systemPackages', [])
        self.all_parts.append(part)
        return part

    def runtime_env(self, root):
        env = []
        env.append("PATH=\"%s/bin:%s/usr/bin:$PATH\"" % (root, root))
        env.append("LD_LIBRARY_PATH=\"%s/lib:%s/usr/lib:$LD_LIBRARY_PATH\"" % (root, root))
        return env

    def build_env(self, root):
        env = []
        env.append("CFLAGS=\"-I%s/include $CFLAGS\"" % root)
        env.append("LDFLAGS=\"-L%s/lib -L%s/usr/lib $LDFLAGS\"" % (root, root))
        return env

    def build_env_for_part(self, part):
        # Grab build env of all part's dependencies

        env = []

        for dep in part.deps:
            root = dep.installdir
            env += self.runtime_env(root)
            env += self.build_env(root)
            env += dep.env(root)
            env += self.build_env_for_part(dep)

        return env

    def stage_env(self):
        root = snapcraft.common.stagedir
        env = []

        env += self.runtime_env(root)
        env += self.build_env(root)
        for part in self.all_parts:
            env += part.env(root)

        return env

    def snap_env(self):
        root = snapcraft.common.snapdir
        env = []

        env += self.runtime_env(root)
        for part in self.all_parts:
            env += part.env(root)

        return env
