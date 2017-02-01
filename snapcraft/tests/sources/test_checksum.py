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

from unittest import mock

class TestChecksum(tests.FakeFileHTTPServerBasedTestCase):

    def setUp(self):
        super().setUp()

    def test_invalid_checksum(self):
        # Create zip file for testing
        os.makedirs(os.path.join('src'))
        file_to_zip = os.path.join('src', 'test.txt')
        open(file_to_zip, 'w').close()
        zip_file = zipfile.ZipFile(os.path.join('src', 'test.zip'), 'w')
        zip_file.write(file_to_zip)
        zip_file.close()

        self.assertRaises(
                ValueError,
                sources.verify_checksum,
                'md6/abcde',
                'src/test.zip')

    def test_correct_checksum(self):
        # Create zip file for testing
        os.makedirs(os.path.join('src'))
        file_to_zip = os.path.join('src', 'test.txt')
        open(file_to_zip, 'w').close()
        zip_file = zipfile.ZipFile(os.path.join('src', 'test.zip'), 'w')
        zip_file.write(file_to_zip)
        zip_file.close()

        calculated_checksum = hashlib.new('md5', \
               open(os.path.join('src', 'test.zip'), 'rb').read())

        source_checksum = print('md5/%r' % calculated_checksum)
        sources.verify_checksum(source_checksum, 'src/test.zip')

    def test_incorrect_checksum(self):
        # Create zip file for testing
        os.makedirs(os.path.join('src'))
        file_to_zip = os.path.join('src', 'test.txt')
        open(file_to_zip, 'w').close()
        zip_file = zipfile.ZipFile(os.path.join('src', 'test.zip'), 'w')
        zip_file.write(file_to_zip)
        zip_file.close()

        incorrect_checksum = 'md5/fe049cfba688aa1af88bc78191d7f904'

        raised = self.assertRaises(
            sources.errors.DigestDoesNotMatchError,
            sources.verify_checksum,
            incorrect_checksum,
            'src/test.zip')

        calculated_checksum = hashlib.new('md5', \
               open(os.path.join('src', 'test.zip'), 'rb').read())
        expected_message= print('Expected the digest for source to be %r, '
            'but it was %r' % (incorrect_checksum, calculated_checksum))

        self.assertEqual(raised.fmt, expected_message)
