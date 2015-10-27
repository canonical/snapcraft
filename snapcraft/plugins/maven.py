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

"""This plugin is useful for building parts that use maven.

The maven build system is commonly used to build Java projects.
The plugin requires a pom.xml in the root of the source tree.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin specific keywords:

    - maven-options:
      (list of strings)
      flags to pass to the build using the maven semantics for parameters.
"""

import glob
import logging
import os

import snapcraft
import snapcraft.common
import snapcraft.plugins.jdk


logger = logging.getLogger(__name__)


class MavenPlugin(snapcraft.plugins.jdk.JdkPlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['maven-options'] = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': True,
            'items': {
                'type': 'string',
            },
            'default': [],
        }

        return schema

    def __init__(self, name, options):
        super().__init__(name, options)
        self.build_packages.append('maven')

    def build(self):
        super().build()

        self.run(['mvn', 'package'] + self.options.maven_options)
        jarfiles = glob.glob(os.path.join(self.builddir, 'target', '*.jar'))
        warfiles = glob.glob(os.path.join(self.builddir, 'target', '*.war'))
        if not (jarfiles or warfiles):
            raise RuntimeError('could not find any built jar files for part')
        if jarfiles:
            jardir = os.path.join(self.installdir, 'jar')
            os.makedirs(jardir, exist_ok=True)
            self.run(['cp', '-a'] + jarfiles + [jardir])
        if warfiles:
            wardir = os.path.join(self.installdir, 'war')
            os.makedirs(wardir, exist_ok=True)
            self.run(['cp', '-a'] + warfiles + [wardir])
