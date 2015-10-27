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

"""The autotools plugin is used for autotools based parts.

Autotools based projects are the ones that have the usual
`./configure && make && make install` instruction set.

The plugin tries to build using ./configure first, if it is not there
it will run ./autogen and if autogen is not there it will run autoreconf.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

In additon, this plugin uses the following plugin specific keywords:

    - configflags:
      (list of strings)
      configure flags to pass to the build such as those shown by running
      './configure --help'
"""

import os
from snapcraft.plugins.make import MakePlugin


class AutotoolsPlugin(MakePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()
        schema['properties']['configflags'] = {
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
        self.build_packages.extend([
            'autoconf',
            'automake',
            'autopoint',
            'libtool',
        ])

    def build(self):
        if not os.path.exists(os.path.join(self.builddir, "configure")):
            if os.path.exists(os.path.join(self.builddir, "autogen.sh")):
                self.run(['env', 'NOCONFIGURE=1', './autogen.sh'],
                         cwd=self.builddir)
            else:
                self.run(['autoreconf', '-i'], cwd=self.builddir)
        self.run(['./configure', '--prefix='] +
                 self.options.configflags)
        super().build()
