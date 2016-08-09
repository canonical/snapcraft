
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

import os

from unittest import mock

import snapcraft
from snapcraft.plugins import cmake
from snapcraft import tests


class CMakeTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        self.project_options = snapcraft.ProjectOptions()

        patcher = mock.patch('snapcraft.internal.common.run')
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('sys.stdout')
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_build_referencing_sourcedir_if_no_subdir(self):
        class Options:
            configflags = []

        plugin = cmake.CMakePlugin('test-part', Options(),
                                   self.project_options)
        os.makedirs(plugin.builddir)
        plugin.build()

        self.run_mock.assert_has_calls([
            mock.call(['cmake', plugin.sourcedir, '-DCMAKE_INSTALL_PREFIX='],
                      cwd=plugin.builddir, env=mock.ANY),
            mock.call(['make', '-j2'], cwd=plugin.builddir, env=mock.ANY),
            mock.call(['make', 'install',
                       'DESTDIR={}'.format(plugin.installdir)],
                      cwd=plugin.builddir, env=mock.ANY)])

    def test_build_referencing_sourcedir_with_subdir(self):
        class Options:
            configflags = []
            source_subdir = 'subdir'

        plugin = cmake.CMakePlugin('test-part', Options(),
                                   self.project_options)
        os.makedirs(plugin.builddir)
        plugin.build()

        sourcedir = os.path.join(
            plugin.sourcedir, plugin.options.source_subdir)
        self.run_mock.assert_has_calls([
            mock.call(['cmake', sourcedir, '-DCMAKE_INSTALL_PREFIX='],
                      cwd=plugin.builddir, env=mock.ANY),
            mock.call(['make', '-j2'], cwd=plugin.builddir, env=mock.ANY),
            mock.call(['make', 'install',
                       'DESTDIR={}'.format(plugin.installdir)],
                      cwd=plugin.builddir, env=mock.ANY)])

    def test_build_environment(self):
        class Options:
            configflags = []

        plugin = cmake.CMakePlugin('test-part', Options(),
                                   self.project_options)
        os.makedirs(plugin.builddir)
        plugin.build()

        expected = {}

        expected['CMAKE_PREFIX_PATH'] = '$CMAKE_PREFIX_PATH:{}'.format(
            self.stage_dir)
        expected['CMAKE_INCLUDE_PATH'] = '$CMAKE_INCLUDE_PATH:' + ':'.join(
            ['{0}/include', '{0}/usr/include', '{0}/include/{1}',
             '{0}/usr/include/{1}']).format(
                 self.stage_dir,
                 plugin.project.arch_triplet)
        expected['CMAKE_LIBRARY_PATH'] = '$CMAKE_LIBRARY_PATH:' + ':'.join(
            ['{0}/lib', '{0}/usr/lib', '{0}/lib/{1}',
             '{0}/usr/lib/{1}']).format(
                 self.stage_dir,
                 plugin.project.arch_triplet)

        self.assertEqual(3, self.run_mock.call_count)
        for call_args in self.run_mock.call_args_list:
            environment = call_args[1]['env']
            for variable, value in expected.items():
                self.assertTrue(
                    variable in environment,
                    'Expected variable "{}" to be in environment'.format(
                        variable))

                self.assertEqual(environment[variable], value,
                                 'Expected ${}={}, but it was {}'.format(
                                 variable, value, environment[variable]))
