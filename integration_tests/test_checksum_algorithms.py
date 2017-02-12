# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import subprocess

import integration_tests


class ChecksumAlgorithmsTestCase(integration_tests.TestCase):

    def test_checksum_algorithms(self):
        project_dir = 'checksum-algorithms'
        self.run_snapcraft('pull', project_dir)

    def test_checksum_invalid(self):
        project_dir = 'checksum-algorithms-invalid'
        self.assertRaises(subprocess.CalledProcessError,
                          self.run_snapcraft,
                          ['pull', 'checksum-md5'],
                          project_dir)
        self.assertRaises(subprocess.CalledProcessError,
                          self.run_snapcraft,
                          ['pull', 'checksum-sha1'],
                          project_dir)
        self.assertRaises(subprocess.CalledProcessError,
                          self.run_snapcraft,
                          ['pull', 'checksum-sha224'],
                          project_dir)
        self.assertRaises(subprocess.CalledProcessError,
                          self.run_snapcraft,
                          ['pull', 'checksum-sha256'],
                          project_dir)
        self.assertRaises(subprocess.CalledProcessError,
                          self.run_snapcraft,
                          ['pull', 'checksum-sha384'],
                          project_dir)
        self.assertRaises(subprocess.CalledProcessError,
                          self.run_snapcraft,
                          ['pull', 'checksum-sha512'],
                          project_dir)
        self.assertRaises(subprocess.CalledProcessError,
                          self.run_snapcraft,
                          ['pull', 'checksum-sha3-256'],
                          project_dir)
        self.assertRaises(subprocess.CalledProcessError,
                          self.run_snapcraft,
                          ['pull', 'checksum-sha3-384'],
                          project_dir)
        self.assertRaises(subprocess.CalledProcessError,
                          self.run_snapcraft,
                          ['pull', 'checksum-sha3-512'],
                          project_dir)
