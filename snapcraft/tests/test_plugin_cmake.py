
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

from snapcraft.plugins import cmake
from snapcraft import (
    common,
    tests,
)


class CMakeTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        patcher = mock.patch('snapcraft.common.run')
        self.run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('sys.stdout')
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.common.get_parallel_build_count')
        self.get_parallel_build_count_mock = patcher.start()
        self.get_parallel_build_count_mock.return_value = 2
        self.addCleanup(patcher.stop)

    def test_build_referencing_sourcedir_if_no_subdir(self):
        class Options:
            configflags = []

        plugin = cmake.CMakePlugin('test-part', Options())
        os.makedirs(plugin.builddir)
        plugin.build()

        self.get_parallel_build_count_mock.assert_called_with()

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

        plugin = cmake.CMakePlugin('test-part', Options())
        os.makedirs(plugin.builddir)
        plugin.build()

        self.get_parallel_build_count_mock.assert_called_with()

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

        plugin = cmake.CMakePlugin('test-part', Options())
        os.makedirs(plugin.builddir)
        plugin.build()

        expected = {}

        expected['CMAKE_PREFIX_PATH'] = '$CMAKE_PREFIX_PATH:{}'.format(
            common.get_stagedir())
        expected['CMAKE_INCLUDE_PATH'] = '$CMAKE_INCLUDE_PATH:' + ':'.join(
            ['{0}/include', '{0}/usr/include', '{0}/include/{1}',
             '{0}/usr/include/{1}']).format(common.get_stagedir(),
                                            common.get_arch_triplet())
        expected['CMAKE_LIBRARY_PATH'] = '$CMAKE_LIBRARY_PATH:' + ':'.join(
            ['{0}/lib', '{0}/usr/lib', '{0}/lib/{1}',
             '{0}/usr/lib/{1}']).format(common.get_stagedir(),
                                        common.get_arch_triplet())

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
