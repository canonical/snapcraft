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

import os
import platform
import re
from unittest.mock import patch

import testtools
from testtools.matchers import (
    Contains,
)

from snapcraft import (
    common,
    tests
)


class CommonTestCase(tests.TestCase):

    def test_get_stagedir(self):
        self.assertEqual(
            os.path.join(self.path, 'stage'), common.get_stagedir())

    def test_get_snapdir(self):
        self.assertEqual(
            os.path.join(self.path, 'snap'), common.get_snapdir())

    def test_get_default_plugindir(self):
        self.assertEqual(
            '/usr/share/snapcraft/plugins', common.get_plugindir())

    def test_set_plugindir(self):
        plugindir = os.path.join(self.path, 'testplugin')
        common.set_plugindir(plugindir)
        self.assertEqual(plugindir, common.get_plugindir())

    def test_isurl(self):
        self.assertTrue(common.isurl('git://'))
        self.assertTrue(common.isurl('bzr://'))
        self.assertFalse(common.isurl('./'))
        self.assertFalse(common.isurl('/foo'))
        self.assertFalse(common.isurl('/fo:o'))

    def test_replace_in_file(self):
        os.makedirs('bin')

        # Place a few files with bad shebangs, and some files that shouldn't be
        # changed.
        files = [
            {
                'path': os.path.join('bin', '2to3'),
                'contents': '#!/foo/bar/baz/python',
                'expected': '#!/usr/bin/env python',
            },
            {
                'path': os.path.join('bin', 'snapcraft'),
                'contents': '#!/foo/baz/python',
                'expected': '#!/usr/bin/env python',
            },
            {
                'path': os.path.join('bin', 'foo'),
                'contents': 'foo',
                'expected': 'foo',
            }
        ]

        for file_info in files:
            with self.subTest(key=file_info['path']):
                with open(file_info['path'], 'w') as f:
                    f.write(file_info['contents'])

                common.replace_in_file('bin', re.compile(r''),
                                       re.compile(r'#!.*python'),
                                       r'#!/usr/bin/env python')

                with open(file_info['path'], 'r') as f:
                    self.assertEqual(f.read(), file_info['expected'])

    @patch('multiprocessing.cpu_count')
    def test_get_parallel_build_count(self, mock_cpu_count):
        mock_cpu_count.return_value = 3
        self.assertEqual(common.get_parallel_build_count(), 3)

    @patch('multiprocessing.cpu_count')
    def test_get_parallel_build_count_disabled(self, mock_cpu_count):
        common.set_enable_parallel_builds(False)
        mock_cpu_count.return_value = 3
        self.assertEqual(common.get_parallel_build_count(), 1)

    @patch('multiprocessing.cpu_count')
    def test_get_parallel_build_count_handle_exception(self, mock_cpu_count):
        mock_cpu_count.side_effect = NotImplementedError
        self.assertEqual(common.get_parallel_build_count(), 1)


class ArchTestCase(testtools.TestCase):

    def setUp(self):
        super().setUp()
        common.host_machine = platform.machine()
        common.target_machine = common.host_machine

    def test_get_arch_with_no_errors(self):
        common.get_arch()

    def test_get_arch_raises_exception_on_non_supported_arch(self):
        common.target_machine = 'badarch'
        e = self.assertRaises(
            common.PlatformError, common.get_arch)
        self.assertEqual(
            'badarch is not supported, please log a bug at '
            'https://bugs.launchpad.net/snapcraft/+filebug?'
            'field.title=please+add+support+for+badarch', str(e))

    def test_get_arch_triplet(self):
        self.assertThat(common.get_arch_triplet(), Contains('linux-gnu'))

    def test_get_arch_triple_raises_exception_on_non_supported_arch(self):
        common.target_machine = 'badarch'
        e = self.assertRaises(
            common.PlatformError, common.get_arch_triplet)
        self.assertEqual(
            'badarch is not supported, please log a bug at '
            'https://bugs.launchpad.net/snapcraft/+filebug?'
            'field.title=please+add+support+for+badarch', str(e))
