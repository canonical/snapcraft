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

"""The ant plugin is useful for ant based parts.

The ant build system is commonly used to build Java projects.
The plugin requires a build.xml in the root of the source tree.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - properties:
      (object)
      A dictionary of key-value pairs. Set the following properties when
      running ant.

    - target:
      (string)
      Run the given ant target.

    - dest-property:
      (string)
      The ant property for the destination directory.
"""

import glob
import logging
import os

import snapcraft
import snapcraft.common
import snapcraft.plugins.jdk


logger = logging.getLogger(__name__)


class AntPlugin(snapcraft.plugins.jdk.JdkPlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['ant-properties'] = {
            'type': 'object',
            'default': {},
        }
        schema['properties']['ant-build-target'] = {
            'type': 'string',
        }
        schema['properties']['ant-dest-property'] = {
            'type': 'string',
        }
        return schema

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.append('ant')

    def build(self):
        super().build()

        command = ['ant']

        if self.options.ant_build_target:
            command.extend([self.options.ant_build_target])

        if self.options.ant_dest_property:
            destination = '-D{}={}'.format(self.options.ant_dest_property,
                                           self.installdir)
            command.extend([destination])

        for prop, value in self.options.ant_properties.items():
            command.extend(['-D{}={}'.format(prop, value)])

        self.run(command)
        if not self.options.ant_dest_property:
            files = glob.glob(os.path.join(self.builddir, 'target', '*.jar'))
            if not files:
                raise RuntimeError(
                    'Could not find any built jar files for part')
            jardir = os.path.join(self.installdir, 'jar')
            os.makedirs(jardir)
            for f in files:
                base = os.path.basename(f)
                os.link(f, os.path.join(jardir, base))

    def env(self, root):
        env = super().env(root)
        jars = glob.glob(os.path.join(self.installdir, 'jar', '*.jar'))
        if jars:
            jars = [os.path.join(root, 'jar',
                    os.path.basename(x)) for x in jars]
            env.extend(
                ['CLASSPATH={}:$CLASSPATH'.format(':'.join(jars))])
        return env
