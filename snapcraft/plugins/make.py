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

    - make-parameters:
      (list of strings)
      Pass the given parameters to the make command.
"""

import snapcraft
import snapcraft.common


class MakePlugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['makefile'] = {
            'type': 'string',
        }
        schema['properties']['make-parameters'] = {
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
        schema['build-properties'].extend(['makefile', 'make-parameters'])

        return schema

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.append('make')

    def build(self):
        super().build()

        command = ['make']

        if self.options.makefile:
            command.extend(['-f', self.options.makefile])

        if self.options.make_parameters:
            command.extend(self.options.make_parameters)

        self.run(command + ['-j{}'.format(self.project.parallel_build_count)])
        self.run(command + ['install', 'DESTDIR=' + self.installdir])
