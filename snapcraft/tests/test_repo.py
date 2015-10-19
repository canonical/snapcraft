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

import fixtures
import logging
import os
import stat
import tempfile
import unittest.mock

from snapcraft import repo
from snapcraft import tests


class UbuntuTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)
        tempdirObj = tempfile.TemporaryDirectory()
        self.addCleanup(tempdirObj.cleanup)
        self.tempdir = tempdirObj.name

    @unittest.mock.patch('snapcraft.repo._get_geoip_country_code_prefix')
    def test_sources_amd64_vivid(self, mock_cc):
        mock_cc.return_value = 'ar'

        sources_list = repo._format_sources_list(
            repo._DEFAULT_SOURCES, 'amd64', 'vivid')

        expected_sources_list = \
            '''deb http://ar.archive.ubuntu.com/ubuntu/ vivid main restricted
deb http://ar.archive.ubuntu.com/ubuntu/ vivid-updates main restricted
deb http://ar.archive.ubuntu.com/ubuntu/ vivid universe
deb http://ar.archive.ubuntu.com/ubuntu/ vivid-updates universe
deb http://ar.archive.ubuntu.com/ubuntu/ vivid multiverse
deb http://ar.archive.ubuntu.com/ubuntu/ vivid-updates multiverse
deb http://security.ubuntu.com/ubuntu vivid-security main restricted
deb http://security.ubuntu.com/ubuntu vivid-security universe
deb http://security.ubuntu.com/ubuntu vivid-security multiverse
'''
        self.assertEqual(sources_list, expected_sources_list)

    @unittest.mock.patch('snapcraft.repo._get_geoip_country_code_prefix')
    def test_sources_armhf_trusty(self, mock_cc):
        sources_list = repo._format_sources_list(
            repo._DEFAULT_SOURCES, 'armhf', 'trusty')

        expected_sources_list = \
            '''deb http://ports.ubuntu.com/ubuntu-ports/ trusty main restricted
deb http://ports.ubuntu.com/ubuntu-ports/ trusty-updates main restricted
deb http://ports.ubuntu.com/ubuntu-ports/ trusty universe
deb http://ports.ubuntu.com/ubuntu-ports/ trusty-updates universe
deb http://ports.ubuntu.com/ubuntu-ports/ trusty multiverse
deb http://ports.ubuntu.com/ubuntu-ports/ trusty-updates multiverse
deb http://ports.ubuntu.com/ubuntu-ports trusty-security main restricted
deb http://ports.ubuntu.com/ubuntu-ports trusty-security universe
deb http://ports.ubuntu.com/ubuntu-ports trusty-security multiverse
'''
        self.assertEqual(sources_list, expected_sources_list)
        self.assertFalse(mock_cc.called)

    def test_fix_symlinks(self):
        os.makedirs(self.tempdir + '/a')
        open(self.tempdir + '/1', mode='w').close()

        os.symlink('a', self.tempdir + '/rel-to-a')
        os.symlink('/a', self.tempdir + '/abs-to-a')
        os.symlink('/b', self.tempdir + '/abs-to-b')
        os.symlink('1', self.tempdir + '/rel-to-1')
        os.symlink('/1', self.tempdir + '/abs-to-1')

        repo._fix_contents(debdir=self.tempdir)

        self.assertEqual(os.readlink(self.tempdir + '/rel-to-a'), 'a')
        self.assertEqual(os.readlink(self.tempdir + '/abs-to-a'), 'a')
        self.assertEqual(os.readlink(self.tempdir + '/abs-to-b'), '/b')
        self.assertEqual(os.readlink(self.tempdir + '/rel-to-1'), '1')
        self.assertEqual(os.readlink(self.tempdir + '/abs-to-1'), '1')

    def test_fix_suid(self):
        files = {
            'suid_file': (0o4765, 0o0765),
            'guid_file': (0o2777, 0o0777),
            'suid_guid_file': (0o6744, 0o0744),
            'suid_guid_sticky_file': (0o7744, 0o1744),
        }

        for key in files:
            with self.subTest(key=key):
                file = os.path.join(self.tempdir, key)
                open(file, mode='w').close()
                os.chmod(file, files[key][0])

                repo._fix_contents(debdir=self.tempdir)
                self.assertEqual(
                    stat.S_IMODE(os.stat(file).st_mode), files[key][1])
