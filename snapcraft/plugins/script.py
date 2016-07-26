# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

"""The script plugin is useful for building script based parts.
Script based projects use bash commands which drive the build.
This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.
Additionally, this plugin uses the following plugin-specific keyword:
    - script-name:
      (string)
      Run the given script name.
    - stages:
      (list of strings)
      Run the script in the given stages. (default: ['build'])
    - pull-arguments:
      (list of strings)
      Pass the given arguments to the script during the pull stage.
    - build-arguments:
      (list of strings)
      Pass the given arguments to the script during the build stage.
"""

import os
import snapcraft


class ScriptPlugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['script-name'] = {
            'type': 'string',
        }
        schema['properties']['stages'] = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': True,
            'items': {
                'type': 'string',
            },
            'default': ['build'],
        }
        schema['properties']['pull-arguments'] = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': True,
            'items': {
                'type': 'string',
            },
            'default': [],
        }
        schema['properties']['build-arguments'] = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': True,
            'items': {
                'type': 'string',
            },
            'default': [],
        }

        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        schema['required'].extend(['script-name'])
        schema['build-properties'].extend(['stages'])
        schema['build-properties'].extend(['pull-arguments'])
        schema['build-properties'].extend(['build-arguments'])

        return schema

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.append('bash')

    def pull(self):
        super().pull()

        if "pull" in self.options.stages:
            command = [os.path.join(self.sourcedir, self.options.script_name)]
            command.extend(self.options.pull_arguments)

            self.run(command)

    def build(self):
        super().build()

        if "build" in self.options.stages:
            command = [os.path.join(self.sourcedir, self.options.script_name)]
            command.extend(self.options.build_arguments)

            self.run(command + [self.installdir])
