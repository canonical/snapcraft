# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2016 Canonical Ltd
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

import os
import shutil

import snapcraft
import snapcraft.common


class CMakePluginBase(snapcraft.BasePlugin):

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

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return super().get_build_properties() + ['configflags']

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
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

        self.run(['cmake', sourcedir, '-DCMAKE_INSTALL_PREFIX='] +
                 self.configflags(), env=self._build_environment())

        self.runCompilation()

    def _build_environment(self):
        env = os.environ.copy()
        env['CMAKE_PREFIX_PATH'] = '$CMAKE_PREFIX_PATH:{}'.format(
            self.project.stage_dir)
        env['CMAKE_INCLUDE_PATH'] = '$CMAKE_INCLUDE_PATH:' + ':'.join(
            ['{0}/include', '{0}/usr/include', '{0}/include/{1}',
             '{0}/usr/include/{1}']).format(
                 self.project.stage_dir, self.project.arch_triplet)
        env['CMAKE_LIBRARY_PATH'] = '$CMAKE_LIBRARY_PATH:' + ':'.join(
            ['{0}/lib', '{0}/usr/lib', '{0}/lib/{1}',
             '{0}/usr/lib/{1}']).format(
                 self.project.stage_dir, self.project.arch_triplet)

        return env

    def configflags(self):
        return self.options.configflags

    def runCompilation(self):
        raise NotImplementedError()
