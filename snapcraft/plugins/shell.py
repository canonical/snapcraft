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

"""
The shell plugin is useful for projects that use esoteric or one-off build
systems. It can run arbitrary commands to build and install the software.

This plugin uses the common plugin keyword sas well as those for for 'sources'.
For more information check the 'plugins' topic for the former and the 'sources'
topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - build-cmds
      (list of strings)
      The commands to run to build and install the software.
"""

import snapcraft
import snapcraft.common


class ShellPlugin(snapcraft.BasePlugin):
    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['build-cmds'] = {
            'type': 'array',
            'minitems': 1,
            'items': {
                'type': 'string',
            },
            'default': [],
        }
        schema['build-properties'].extend(['build-cmds'])
        return schema

    def __init__(self, name, options, project):
        super().__init__(name, options, project)

    def build(self):
        super().build()
        for cmd in self.options.build_cmds:
            self.run(cmd.split() if ' ' in cmd else [cmd])
