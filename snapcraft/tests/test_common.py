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
import re

from snapcraft.internal import common
from snapcraft import tests


class CommonTestCase(tests.TestCase):

    def test_get_default_plugindir(self):
        self.assertEqual(
            '/usr/share/snapcraft/plugins', common.get_plugindir())

    def test_set_plugindir(self):
        plugindir = os.path.join(self.path, 'testplugin')
        common.set_plugindir(plugindir)
        self.assertEqual(plugindir, common.get_plugindir())

    def test_get_default_examplesdir(self):
        self.assertEqual(
            '/usr/share/snapcraft/examples', common.get_examplesdir())

    def test_set_examplesdir(self):
        examplesdir = os.path.join(self.path, 'testexamples')
        common.set_examplesdir(examplesdir)
        self.assertEqual(examplesdir, common.get_examplesdir())

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


class CommonMigratedTestCase(tests.TestCase):

    def test_parallel_build_count_migration_message(self):
        with self.assertRaises(EnvironmentError) as raised:
            common.get_parallel_build_count()

        self.assertEqual(
            str(raised.exception),
            "This plugin is outdated, use "
            "'project.parallel_build_count'")

    def test_deb_arch_migration_message(self):
        with self.assertRaises(EnvironmentError) as raised:
            common.get_arch()

        self.assertEqual(
            str(raised.exception),
            "This plugin is outdated, use 'project.deb_arch'")

    def test_arch_triplet_migration_message(self):
        with self.assertRaises(EnvironmentError) as raised:
            common.get_arch_triplet()

        self.assertEqual(
            str(raised.exception),
            "This plugin is outdated, use 'project.arch_triplet'")


class FormatInColumnsTestCase(tests.TestCase):

    elements_list = ['ant', 'autotools', 'catkin', 'cmake', 'copy', 'go',
                     'jdk', 'kbuild', 'kernel', 'make', 'maven', 'nil',
                     'nodejs', 'python2', 'python3', 'scons', 'tar-content']

    def test_format_output_in_columns_default(self):
        """Format output on 2 lines, with default max-width and space sep"""
        expected = ['ant        catkin  copy  jdk     kernel  maven  '
                    'nodejs   python3  tar-content',
                    'autotools  cmake   go    kbuild  make    nil    '
                    'python2  scons  ']
        self.assertEquals(expected,
                          common.format_output_in_columns(self.elements_list))

    def test_format_output_in_columns_narrow(self):
        """Format output on 3 lines, with narrow max-width and space sep"""
        expected = ['ant        cmake  jdk     make   nodejs   scons      ',
                    'autotools  copy   kbuild  maven  python2  tar-content',
                    'catkin     go     kernel  nil    python3']
        self.assertEquals(expected,
                          common.format_output_in_columns(self.elements_list,
                                                          max_width=60))

    def test_format_output_in_columns_large(self):
        """Format output on one big line, with default space sep"""
        expected = ['ant  autotools  catkin  cmake  copy  go  jdk  kbuild  '
                    'kernel  make  maven  nil  nodejs  python2  python3  '
                    'scons  tar-content']
        self.assertEquals(expected,
                          common.format_output_in_columns(self.elements_list,
                                                          max_width=990))

    def test_format_output_in_columns_one_space(self):
        """Format output with one space sep"""
        expected = ['ant       cmake jdk    make  nodejs  scons      ',
                    'autotools copy  kbuild maven python2 tar-content',
                    'catkin    go    kernel nil   python3']
        self.assertEquals(expected,
                          common.format_output_in_columns(self.elements_list,
                                                          max_width=60,
                                                          num_col_spaces=1))
