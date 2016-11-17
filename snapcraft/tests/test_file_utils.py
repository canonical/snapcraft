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
import unittest.mock

from snapcraft import file_utils
from snapcraft import tests


class ReplaceInFileTestCase(tests.TestCase):

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

                file_utils.replace_in_file('bin', re.compile(r''),
                                           re.compile(r'#!.*python'),
                                           r'#!/usr/bin/env python')

                with open(file_info['path'], 'r') as f:
                    self.assertEqual(f.read(), file_info['expected'])

    def test_replace_in_file_with_permission_error(self):
        os.makedirs('bin')
        file_info = {
            'path': os.path.join('bin', 'readonly'),
            'contents': '#!/foo/bar/baz/python',
            'expected': '#!/foo/bar/baz/python',
        }
        with open(file_info['path'], 'w') as f:
            f.write(file_info['contents'])

        with unittest.mock.patch('snapcraft.file_utils.open') as mock_open:
            mock_open.side_effect = PermissionError("")
            file_utils.replace_in_file('bin', re.compile(r''),
                                       re.compile(r'#!.*python'),
                                       r'#!/usr/bin/env python')

        with open(file_info['path'], 'r') as f:
            self.assertEqual(f.read(), file_info['expected'])


class TestLinkOrCopyTree(tests.TestCase):

    def setUp(self):
        super().setUp()

        os.makedirs('foo/bar/baz')
        open('1', 'w').close()
        open(os.path.join('foo', '2'), 'w').close()
        open(os.path.join('foo', 'bar', '3'), 'w').close()
        open(os.path.join('foo', 'bar', 'baz', '4'), 'w').close()

    def test_link_file_to_file_raises(self):
        with self.assertRaises(NotADirectoryError) as raised:
            file_utils.link_or_copy_tree('1', 'qux')

        self.assertEqual(str(raised.exception), "'1' is not a directory")

    def test_link_file_into_directory(self):
        os.mkdir('qux')
        with self.assertRaises(NotADirectoryError) as raised:
            file_utils.link_or_copy_tree('1', 'qux')

        self.assertEqual(str(raised.exception), "'1' is not a directory")

    def test_link_directory_to_directory(self):
        file_utils.link_or_copy_tree('foo', 'qux')
        self.assertTrue(os.path.isfile(os.path.join('qux', '2')))
        self.assertTrue(os.path.isfile(os.path.join('qux', 'bar', '3')))
        self.assertTrue(os.path.isfile(os.path.join('qux', 'bar', 'baz', '4')))

    def test_link_directory_overwrite_file_raises(self):
        open('qux', 'w').close()
        with self.assertRaises(NotADirectoryError) as raised:
            file_utils.link_or_copy_tree('foo', 'qux')

        self.assertEqual(
            str(raised.exception),
            "Cannot overwrite non-directory 'qux' with directory 'foo'")

    def test_link_subtree(self):
        file_utils.link_or_copy_tree('foo/bar', 'qux')
        self.assertTrue(os.path.isfile(os.path.join('qux', '3')))
        self.assertTrue(os.path.isfile(os.path.join('qux', 'baz', '4')))


class TestLinkOrCopy(tests.TestCase):

    def setUp(self):
        super().setUp()

        os.makedirs('foo/bar/baz')
        open('1', 'w').close()
        open(os.path.join('foo', '2'), 'w').close()
        open(os.path.join('foo', 'bar', '3'), 'w').close()
        open(os.path.join('foo', 'bar', 'baz', '4'), 'w').close()

    def test_copy_nested_file(self):
        file_utils.link_or_copy('foo/bar/baz/4', 'foo2/bar/baz/4')
        self.assertTrue(os.path.isfile('foo2/bar/baz/4'))
