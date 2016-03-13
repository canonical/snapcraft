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
"""

import os
import shutil

import snapcraft.plugins.make
from snapcraft import common


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

        return schema

    def __init__(self, name, options):
        super().__init__(name, options)
        self.build_packages.append('cmake')

    def build(self):
        if os.path.exists(self.builddir):
            shutil.rmtree(self.builddir)
        os.mkdir(self.builddir)

        source_subdir = getattr(self.options, 'source_subdir', None)
        if source_subdir:
            sourcedir = os.path.join(self.sourcedir, source_subdir)
        else:
            sourcedir = self.sourcedir

        env = self._build_environment()

        self.run(['cmake', sourcedir, '-DCMAKE_INSTALL_PREFIX=',
                  '-DCMAKE_BUILD_TYPE=None', '-DCMAKE_VERBOSE_MAKEFILE=ON'] +
                 self.options.configflags, env=env)

        self.run(['make', '-j{}'.format(common.get_parallel_build_count())],
                 env=env)

        self.run(['make', 'install', 'DESTDIR=' + self.installdir], env=env)

    def _build_environment(self):
        env = os.environ.copy()
        env['CMAKE_PREFIX_PATH'] = '$CMAKE_PREFIX_PATH:{}'.format(
            common.get_stagedir())
        env['CMAKE_INCLUDE_PATH'] = '$CMAKE_INCLUDE_PATH:' + ':'.join(
            ['{0}/include', '{0}/usr/include', '{0}/include/{1}',
             '{0}/usr/include/{1}']).format(common.get_stagedir(),
                                            common.get_arch_triplet())
        env['CMAKE_LIBRARY_PATH'] = '$CMAKE_LIBRARY_PATH:' + ':'.join(
            ['{0}/lib', '{0}/usr/lib', '{0}/lib/{1}',
             '{0}/usr/lib/{1}']).format(common.get_stagedir(),
                                        common.get_arch_triplet())

        return env
