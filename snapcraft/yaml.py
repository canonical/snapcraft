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

import logging
import sys

import yaml
import jsonschema
import os

import snapcraft.plugin
from snapcraft import common


logger = logging.getLogger(__name__)


def _validate_snapcraft_yaml(snapcraft_yaml):
    schema_file = os.path.abspath(os.path.join(common.get_schemadir(), 'snapcraft.yaml'))
    with open(schema_file) as fp:
        schema = yaml.load(fp)

    jsonschema.validate(snapcraft_yaml, schema)


class Config:

    def __init__(self):
        self.build_tools = []
        self.all_parts = []
        afterRequests = {}

        try:
            with open("snapcraft.yaml", 'r') as fp:
                self.data = yaml.load(fp)
        except FileNotFoundError:
            logger.error("Could not find snapcraft.yaml.  Are you sure you're in the right directory?\nTo start a new project, use 'snapcraft init'")
            sys.exit(1)

        # Make sure the loaded snapcraft yaml follows the schema
        try:
            _validate_snapcraft_yaml(self.data)
        except FileNotFoundError:
            logger.error('Schema is missing, could not validate snapcraft.yaml, check installation')
            sys.exit(1)
        except jsonschema.ValidationError as e:
            msg = "Issues while validating snapcraft.yaml: {}".format(e.message)
            logger.error(msg)
            sys.exit(1)

        self.build_tools = self.data.get('build-tools', [])

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
                    logger.error("Could not find part name %s" % dep)
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
                logger.error("Circular dependency chain!")
                sys.exit(1)
            sortedParts = [topPart] + sortedParts
            self.all_parts.remove(topPart)
        self.all_parts = sortedParts

    def load_plugin(self, part_name, plugin_name, properties, load_code=True):
        part = snapcraft.plugin.load_plugin(part_name, plugin_name, properties, load_code=load_code)

        self.build_tools += part.config.get('build-tools', [])
        self.all_parts.append(part)
        return part

    def runtime_env(self, root):
        env = []
        env.append('PATH="{0}/bin:{0}/usr/bin:$PATH"'.format(root))
        env.append('LD_LIBRARY_PATH="{0}/lib:{0}/usr/lib:{0}/lib/{1}:{0}/usr/lib/{1}:$LD_LIBRARY_PATH"'.format(root, snapcraft.common.get_arch_triplet()))
        return env

    def build_env(self, root):
        env = []
        env.append('CFLAGS="-I{0}/include -I{0}/usr/include -I{0}/include/{1} -I{0}/usr/include/{1} $CFLAGS"'.format(root, snapcraft.common.get_arch_triplet()))
        env.append('LDFLAGS="-L{0}/lib -L{0}/usr/lib -L{0}/lib/{1} -L{0}/usr/lib/{1} $LDFLAGS"'.format(root, snapcraft.common.get_arch_triplet()))
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
        root = common.get_stagedir()
        env = []

        env += self.runtime_env(root)
        env += self.build_env(root)
        for part in self.all_parts:
            env += part.env(root)

        return env

    def snap_env(self):
        root = common.get_snapdir()
        env = []

        env += self.runtime_env(root)
        for part in self.all_parts:
            env += part.env(root)

        return env
