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

"""The cmake plugin is useful for building cmake based parts.

These are projects that have a CMakeLists.txt that drives the build.
The plugin requires a CMakeLists.txt in the root of the source tree.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - configflags:
      (list of strings)
      configure flags to pass to the build using the common cmake semantics.
    - install-via:
      (enum, 'destdir' or 'prefix'; default 'destdir')
      Whether to install via DESTDIR or CMAKE_INSTALL_PREFIX
"""

import os
import shutil

import snapcraft.plugins.make


class CMakePlugin(snapcraft.plugins.make.MakePlugin):

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

        schema['properties']['install-via'] = {
            'enum': ['destdir', 'prefix'],
            'default': 'destdir',
        }

        return schema

    def __init__(self, name, options):
        super().__init__(name, options)
        self.build_packages.append('cmake')

        if options.install_via == 'destdir':
            self.install_via_destdir = True
        elif options.install_via == 'prefix':
            self.install_via_destdir = False
        else:
            raise RuntimeError('Unsupported installation method: "{}"'.format(
                options.install_via))

    def build(self):
        if os.path.exists(self.builddir):
            shutil.rmtree(self.builddir)
        os.mkdir(self.builddir)

        source_subdir = getattr(self.options, 'source_subdir', None)
        if source_subdir:
            sourcedir = os.path.join(self.sourcedir, source_subdir)
        else:
            sourcedir = self.sourcedir

        cmake_command = ['cmake', sourcedir]
        make_command = ['make', 'install']

        if self.install_via_destdir:
            cmake_command.append('-DCMAKE_INSTALL_PREFIX=')
            make_command.append('DESTDIR=' + self.installdir)
        else:
            cmake_command.append('-DCMAKE_INSTALL_PREFIX={}'.format(
                self.installdir))

        self.run(cmake_command + self.options.configflags)
        self.run(make_command)
