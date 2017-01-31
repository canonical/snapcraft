# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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
import tarfile
import fixtures
import unittest
import hashlib
import zipfile
import libarchive

from snapcraft.internal import sources

from snapcraft import tests

#from snapcraft.tests.sources import SourceTestCase
from unittest import mock

class TestChecksum(tests.FakeFileHTTPServerBasedTestCase):

    def setUp(self):
        super().setUp()

    def test_tarball_with_wrong_checksum(self):
        # Create tar file for testing
        os.makedirs(os.path.join('src'))
        file_to_tar = os.path.join('src', 'test.txt')
        open(file_to_tar, 'w').close()
        tar = tarfile.open(os.path.join('src', 'test.tar'), 'w')
        tar.add(file_to_tar)
        tar.close()

        tar_source = sources.Tar(os.path.join('src', 'test.tar'), 'dst',
                source_checksum = 'md6/xxxxxxxxxxxxxxxxxxxxxxxxxx')
        os.mkdir('dst')

        #raised =
        self.assertRaises(ValueError, tar_source.pull)

        #self.assertEqual(raised.message, 'unsupported hash type')

    def test_tarball_with_correct_checksum(self):
        # Create tar file for testing
        os.makedirs(os.path.join('src'))
        file_to_tar = os.path.join('src', 'test.txt')
        open(file_to_tar, 'w').close()
        tar = tarfile.open(os.path.join('src', 'test.tar'), 'w')
        tar.add(file_to_tar)
        tar.close()

        calculated_checksum = hashlib.new('md5', \
               open(os.path.join('src', 'test.tar'), 'rb').read())
        source_checksum = print('md5/%r' % calculated_checksum)

        tar_source = sources.Tar(os.path.join('src', 'test.tar'), 'dst',
                source_checksum = source_checksum)
        os.mkdir('dst')

        tar_source.pull()

    @mock.patch('zipfile.ZipFile')
    def test_zip_with_wrong_checksum(self, mock_zip):
        dest_dir = 'src'
        os.makedirs(dest_dir)
        zip_file_name = 'test.zip'
        source = 'http://{}:{}/{file_name}'.format(
            *self.server.server_address, file_name=zip_file_name)
        zip_source = sources.Zip(source, dest_dir,
                source_checksum = 'md6/abcde')

        self.assertRaises(ValueError, zip_source.pull)

    def test_zip_with_correct_checksum(self):
        # Create zip file for testing
        os.makedirs(os.path.join('src'))
        file_to_zip = os.path.join('src', 'test.txt')
        open(file_to_zip, 'w').close()
        zip_file = zipfile.ZipFile(os.path.join('src', 'test.zip'), 'w')
        zip_file.write(file_to_zip)
        zip_file.close()

        dest_dir = 'dst'
        os.makedirs(dest_dir)
        calculated_checksum = hashlib.new('md5', \
               open(os.path.join('src', 'test.zip'), 'rb').read())
        source_checksum = print('md5/%r' % calculated_checksum)

        zip_source = sources.Zip(os.path.join('src', 'test.zip'), dest_dir,
                source_checksum = source_checksum)

        zip_source.pull()

    @mock.patch('debian.debfile.DebFile')
    def test_deb_with_wrong_checksum(self, mock_deb):
        dest_dir = 'src'
        os.makedirs(dest_dir)
        deb_file_name = 'test.deb'
        source = 'http://{}:{}/{file_name}'.format(
            *self.server.server_address, file_name=deb_file_name)
        deb_source = sources.Deb(source, dest_dir,
                source_checksum = 'md6/abcde')

        self.assertRaises(ValueError, deb_source.pull)

    def test_rpm_with_wrong_checksum(self):
        # Create rpm file for testing
        rpm_file_name = 'test.rpm'
        dest_dir = 'src'
        os.makedirs(dest_dir)
        test_file_path = os.path.join(self.path, 'test.txt')
        open(test_file_path, 'w').close()
        rpm_file_path = os.path.join(self.path, rpm_file_name)
        os.chdir(self.path)
        with libarchive.file_writer(rpm_file_path, 'cpio', 'gzip') as rpm:
            rpm.add_files('test.txt')

        rpm_source = sources.Rpm(rpm_file_path, dest_dir,
                source_checksum = 'md6/abcde')

        self.assertRaises(ValueError, rpm_source.pull)

    def test_rpm_with_correct_checksum(self):
        # Create rpm file for testing
        rpm_file_name = 'test.rpm'
        dest_dir = 'src'
        os.makedirs(dest_dir)
        test_file_path = os.path.join(self.path, 'test.txt')
        open(test_file_path, 'w').close()
        rpm_file_path = os.path.join(self.path, rpm_file_name)
        os.chdir(self.path)
        with libarchive.file_writer(rpm_file_path, 'cpio', 'gzip') as rpm:
            rpm.add_files('test.txt')

        calculated_checksum = hashlib.new('md5', \
               open(os.path.join(rpm_file_name), 'rb').read())
        source_checksum = print('md5/%r' % calculated_checksum)

        rpm_source = sources.Rpm(rpm_file_path, dest_dir,
                source_checksum = source_checksum)

        rpm_source.pull()
