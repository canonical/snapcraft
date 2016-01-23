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
import os.path
from unittest.mock import Mock

from snapcraft.plugins.copy import CopyPlugin
from snapcraft.tests import TestCase


class TestCopyPlugin(TestCase):

    def setUp(self):
        super().setUp()
        self.mock_options = Mock()
        self.mock_options.files = {}
        # setup the expected target dir in our tempdir
        self.dst_prefix = 'parts/copy/install/'
        os.makedirs(self.dst_prefix)

    def test_copy_plugin_with_follow_symlink_true_copies_symlinked_file(self):
        self.mock_options.files = {
            'src': 'dst'
        }

        self.mock_options.follow_symlinks = True

        os.makedirs('src')
        open('src/file.txt', 'w').close()
        os.symlink('file.txt', 'src/link.txt')

        c = CopyPlugin('copy', self.mock_options)
        c.build()

        self.assertTrue(
            os.path.isfile(os.path.join(self.dst_prefix, 'dst/link.txt')))
        self.assertFalse(
            os.path.islink(os.path.join(self.dst_prefix, 'dst/link.txt')))

    def test_copy_plugin_with_follow_symlink_false_copies_symlink_itself(self):
        self.mock_options.files = {
            'src': 'dst'
        }

        self.mock_options.follow_symlinks = False

        os.makedirs('src')
        open('src/file.txt', 'w').close()
        os.symlink('file.txt', 'src/link.txt')

        c = CopyPlugin('copy', self.mock_options)
        c.build()

        self.assertTrue(
            os.path.islink(os.path.join(self.dst_prefix, 'dst/link.txt')))

    def test_copy_plugin_any_missing_src_raises_exception(self):
        # ensure that a bad file causes a warning and fails the build even
        # if there is a good file last
        self.mock_options.files = {
            'src': 'dst',
            'zzz': 'zzz',
        }
        open('zzz', 'w').close()
        c = CopyPlugin('copy', self.mock_options)

        with self.assertRaises(EnvironmentError) as raised:
            c.build()

        self.assertEqual(raised.exception.__str__(),
                         "[Errno 2] No such file or directory: 'src'")

    def test_copy_glob_does_not_match_anything(self):
        # ensure that a bad file causes a warning and fails the build even
        # if there is a good file last
        self.mock_options.files = {
            'src*': 'dst',
        }
        c = CopyPlugin('copy', self.mock_options)

        with self.assertRaises(EnvironmentError) as raised:
            c.build()

        self.assertEqual(raised.exception.__str__(), "no matches for 'src*'")

    def test_copy_plugin_copies(self):
        self.mock_options.files = {
            'src': 'dst',
        }
        open('src', 'w').close()

        c = CopyPlugin('copy', self.mock_options)
        c.build()
        self.assertTrue(os.path.exists(os.path.join(self.dst_prefix, 'dst')))

    def test_copy_plugin_creates_prefixes(self):
        self.mock_options.files = {
            'src': 'dir/dst',
        }
        open('src', 'w').close()

        c = CopyPlugin('copy', self.mock_options)
        c.build()
        self.assertTrue(os.path.exists(os.path.join(self.dst_prefix,
                                                    'dir/dst')))

    def test_copy_directories(self):
        self.mock_options.files = {
            'dirs1': 'dir/dst',
        }
        os.mkdir('dirs1')
        file = os.path.join('dirs1', 'f')
        open(file, 'w').close()

        c = CopyPlugin('copy', self.mock_options)
        c.build()
        self.assertTrue(
            os.path.exists(os.path.join(self.dst_prefix, 'dir', 'dst', 'f')))

    def test_copy_plugin_glob(self):
        self.mock_options.files = {
            '*.txt': '.',
        }

        for filename in ('file-a.txt', 'file-b.txt', 'file-c.notxt'):
            with open(filename, 'w') as datafile:
                datafile.write(filename)

        c = CopyPlugin('copy', self.mock_options)
        c.build()

        self.assertTrue(os.path.exists(
            os.path.join(self.dst_prefix, 'file-a.txt')))
        self.assertTrue(os.path.exists(
            os.path.join(self.dst_prefix, 'file-b.txt')))
        self.assertFalse(os.path.exists(
            os.path.join(self.dst_prefix, 'file-c.notxt')))
