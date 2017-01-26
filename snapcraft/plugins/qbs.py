# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Dan Chapman <dpniel@ubuntu.com>
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

"""The QBS plugin is useful for building parts that use the Qt Build Suite.

These are projects that are built using .qbs files and are using gcc or
clang based toolchains.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - qbs-build-variant:
      (enum, 'debug' or 'release')
      Set either debug or release build variant. Default is release.
    - qbs-options:
      (list of strings)
      Additional options to pass to the qbs invocation.
    - qbs-profile:
      (enum, 'gcc' or 'clang')
      Set either gcc or clang as the base build profile. Default is gcc.
    - qt-version:
      (enum, 'qt4' or 'qt5')
      Optionally you can set up profiles for qt4 and qt5 applications.
      If no qt version is set then the base gcc/clang profile will be used.
"""
import os
import shutil
import snapcraft


class QbsPlugin(snapcraft.BasePlugin):

    @classmethod
    def schema(cls):
        schema = super().schema()

        schema['properties']['qbs-build-variant'] = {
            'enum': ['debug', 'release'],
            'default': 'release',
        }

        schema['properties']['qbs-options'] = {
            'type': 'array',
            'minitems': 1,
            'uniqueItems': True,
            'items': {
                'type': 'string',
            },
            'default': [],
        }

        schema['properties']['qbs-profile'] = {
            'enum': ['gcc', 'clang'],
            'default': 'gcc',
        }

        schema['properties']['qt-version'] = {
            'enum': ['qt4', 'qt5'],
            'default': None
        }

        return schema

    @classmethod
    def get_pull_properties(cls):
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the pull step dirty.
        return ['qt-version', 'qbs-profile']

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ['qbs-options', 'qbs-build-variant']

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.extend(['qbs', 'gcc', 'clang'])

        if self.options.qt_version == 'qt5':
            self.build_packages.extend(['qt5-qmake', 'qtbase5-dev'])
        elif self.options.qt_version == 'qt4':
            self.build_packages.extend(['qt4-qmake', 'libqt4-dev'])
        elif self.options.qt_version is not None:
            raise RuntimeError('Unsupported Qt version: {!r}'.format(
                self.options.qt_version))

    def build(self):
        super().build()

        env = self._build_environment()

        # Setup the toolchains, there will only be gcc or clang by default
        self.run(['qbs', 'setup-toolchains', '--detect'], env=env)

        build_profile = ''

        if self.options.qt_version is None:
            build_profile = self.options.qbs_profile
        else:
            # a unique'ish name for snap builds. Hopefully this shouldn't clash
            # with any other local profiles. Each profile should look something
            # like: profiles.snapcraft-qbs-qt5-clang
            build_profile = 'snapcraft-qbs-{}-{}'.format(
                self.options.qt_version,
                self.options.qbs_profile)

            # Find full qmake path as qbs setup-qt requires it
            qmake = shutil.which('qmake')
            if qmake is None:
                raise RuntimeError('The qbs plugin requires qmake be on PATH')

            # Setup the Qt profile.
            self.run(['qbs', 'setup-qt', qmake, build_profile], env=env)

            # Switch buildprofile to clang if required
            # we don't need to set gcc as that is the default baseProfile
            if self.options.qbs_profile == 'clang':
                self.run(['qbs', 'config',
                          'profiles.{}.baseProfile'.format(build_profile),
                          self.options.qbs_profile],
                         env=env)

        # Run the build.
        self.run(['qbs', 'build',
                  '-d', self.builddir,
                  '-f', self.sourcedir,
                  self.options.qbs_build_variant,
                  'qbs.installRoot:' + self.installdir,
                  'profile:' + build_profile] + self.options.qbs_options,
                 env=env)

    def _build_environment(self):
        env = os.environ.copy()
        if self.options.qt_version is not None:
            env['QT_SELECT'] = self.options.qt_version
        return env
