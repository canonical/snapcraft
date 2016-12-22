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

    - ant-properties:
      (object)
      A dictionary of key-value pairs. Set the following properties when
      running ant.

    - ant-build-targets:
      (list of strings)
      Run the given ant targets.
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
        schema['properties']['ant-build-targets'] = {
            'type': 'array',
            'uniqueItems': True,
            'items': {
                'type': 'string',
            },
            'default': [],
        }
        return schema

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.append('ant')

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ['ant-build-targets', 'ant-properties']

    def build(self):
        super().build()

        command = ['ant']

        if self.options.ant_build_targets:
            command.extend(self.options.ant_build_targets)

        for prop, value in self.options.ant_properties.items():
            command.extend(['-D{}={}'.format(prop, value)])

        self.run(command)
        files = glob.glob(os.path.join(self.builddir, 'target', '*.jar'))
        if files:
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
