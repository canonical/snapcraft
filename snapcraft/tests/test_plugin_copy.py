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

import os.path
import tarfile
from unittest.mock import Mock

from snapcraft.plugins.copy import CopyPlugin
from snapcraft.tests import TestCase


class TestCopyPlugin(TestCase):

    def setUp(self):
        super().setUp()
        self.mock_options = Mock()
        self.mock_options.files = {}
        self.mock_options.destination = ''
        self.mock_options.source = ''
        # setup the expected src and target dir in our tempdir
        self.src_prefix = 'parts/copy/src/'
        os.makedirs(self.src_prefix)
        self.build_prefix = 'parts/copy/build/'
        os.makedirs(self.build_prefix)
        self.install_prefix = 'parts/copy/install/'
        os.makedirs(self.install_prefix)

    def test_copy_plugin_any_missing_src_raises_exception(self):
        # ensure that a bad file causes a warning and fails the build even
        # if there is a good file last
        self.mock_options.files = {
            'src': 'dst',
            'zzz': 'zzz',
        }
        open('zzz', 'w').close()
        c = CopyPlugin('copy', self.mock_options)
        c.pull()

        with self.assertRaises(EnvironmentError) as raised:
            c.build()

        self.assertEqual(raised.exception.__str__(),
                         "[Errno 2] No such file or directory: '{}'".format(
                         os.path.join(os.getcwd(), self.src_prefix, 'src')))

    def test_copy_glob_does_not_match_anything(self):
        # ensure that a bad file causes a warning and fails the build even
        # if there is a good file last
        self.mock_options.files = {
            'src*': 'dst',
        }
        c = CopyPlugin('copy', self.mock_options)
        c.pull()

        with self.assertRaises(EnvironmentError) as raised:
            c.build()

        self.assertEqual(raised.exception.__str__(),
                         "no matches for '{}'".format(
                         os.path.join(os.getcwd(), self.src_prefix, 'src*')))

    def test_copy_plugin_copies(self):
        self.mock_options.files = {
            'src': 'dst',
        }
        open('src', 'w').close()

        c = CopyPlugin('copy', self.mock_options)
        c.pull()
        c.build()
        self.assertTrue(os.path.exists(os.path.join(self.install_prefix,
                                                    'dst')))

    def test_copy_plugin_creates_prefixes(self):
        self.mock_options.files = {
            'src': 'dir/dst',
        }
        open('src', 'w').close()

        c = CopyPlugin('copy', self.mock_options)
        c.pull()
        c.build()
        self.assertTrue(os.path.exists(os.path.join(self.install_prefix,
                                                    'dir/dst')))

    def test_copy_directories(self):
        self.mock_options.files = {
            'dirs1': 'dir/dst',
        }
        os.mkdir('dirs1')
        file = os.path.join('dirs1', 'f')
        open(file, 'w').close()

        c = CopyPlugin('copy', self.mock_options)
        c.pull()
        c.build()
        self.assertTrue(
            os.path.exists(os.path.join(self.install_prefix, 'dir',
                                        'dst', 'f')))

    def test_copy_plugin_glob(self):
        self.mock_options.files = {
            '*.txt': '.',
        }

        for filename in ('file-a.txt', 'file-b.txt', 'file-c.notxt'):
            with open(filename, 'w') as datafile:
                datafile.write(filename)

        c = CopyPlugin('copy', self.mock_options)
        c.pull()
        c.build()

        self.assertTrue(os.path.exists(
            os.path.join(self.install_prefix, 'file-a.txt')))
        self.assertTrue(os.path.exists(
            os.path.join(self.install_prefix, 'file-b.txt')))
        self.assertFalse(os.path.exists(
            os.path.join(self.install_prefix, 'file-c.notxt')))

    def test_dest_abs_path_raises_exception(self):
        class Options:
            source = '.'
            destination = '/destdir1'
        # ensure that a absolute path for a destination directory
        # raises an exception
        with self.assertRaises(ValueError) as raised:
            CopyPlugin('copy', Options())

        self.assertEqual(raised.exception.__str__(),
                         'path "/destdir1" must be relative')

    def test_install_destination_dir_exists(self):
        class Options:
            source = os.path.join('src', 'test.tar')
            destination = 'destdir1'

        # create tar file for testing
        os.mkdir('src')
        file_to_tar = os.path.join('src', 'test.txt')
        open(file_to_tar, 'w').close()
        tar = tarfile.open(os.path.join('src', 'test.tar'), "w")
        tar.add(file_to_tar)
        tar.close()

        c = CopyPlugin('copy', Options())
        c.build()

        self.assertTrue(
            os.path.exists(os.path.join(self.install_prefix, 'destdir1')))
        self.assertTrue(
            os.path.exists(
                os.path.join(self.install_prefix, 'destdir1', 'test.txt')))

    def test_without_destination_dir_attribute_defined(self):
        class Options:
            source = '.'
            destination = None
        c = CopyPlugin('copy', Options())
        c.pull()
        c.build()

        self.assertTrue(
            os.path.exists(os.path.join(self.build_prefix)))
