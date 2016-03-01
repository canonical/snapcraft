# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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

"""The make plugin is useful for building make based parts.

Make based projects are projects that have a Makefile that drives the
build.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keyword:

    - makefile:
      (string)
      Use the given file as the makefile.
"""

import snapcraft
import snapcraft.common


class MakePlugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['makefile'] = {
            'type': 'string'
        }

        return schema

    def __init__(self, name, options):
        super().__init__(name, options)
        self.build_packages.append('make')

    def build(self):
        super().build()

        command = ['make']

        if self.options.makefile:
            command.extend(['-f', self.options.makefile])

        self.run(command + ['-j{}'.format(
            snapcraft.common.get_parallel_build_count())])
        self.run(command + ['install', 'DESTDIR=' + self.installdir])
