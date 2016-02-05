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

import fixtures
import pexpect

import integration_tests


class LoginLogoutTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()
        self.useFixture(fixtures.EnvironmentVariable(
            'UBUNTU_STORE_API_ROOT_URL',
            'https://myapps.developer.staging.ubuntu.com/dev/api/'))
        self.useFixture(fixtures.EnvironmentVariable(
            'UBUNTU_STORE_UPLOAD_ROOT_URL',
            'https://upload.apps.staging.ubuntu.com/'))
        self.useFixture(fixtures.EnvironmentVariable(
            'UBUNTU_SSO_API_ROOT_URL',
            'https://login.staging.ubuntu.com/api/v2/'))

    def _login(self, email, password, expect_success=True):
        process = pexpect.spawn(self.snapcraft_command, ['login'])
        process.expect_exact(
            'Enter your Ubuntu One SSO credentials.\r\n'
            'Email: ')
        process.sendline(email)
        process.expect_exact('Password: ')
        process.sendline(password)
        process.expect_exact(
            "One-time password (just press enter if you don't use two-factor "
            "authentication): ")
        process.sendline('')
        process.expect_exact(
            # bold.
            '\r\n\x1b[1m'
            'Authenticating against Ubuntu One SSO.'
            '\x1b[0m\r\n')
        result = 'successful' if expect_success else 'failed'
        process.expect_exact(
            # bold.
            '\x1b[1m'
            'Login {}.'.format(result) +
            '\x1b[0m\r\n')

    def _logout(self):
        output = self.run_snapcraft('logout')
        expected = ('Clearing credentials for Ubuntu One SSO.\n'
                    'Credentials cleared.\n')
        self.assertEqual(expected, output)

    def test_successful_login(self):
        self.addCleanup(self._logout)
        password = os.getenv('TEST_USER_PASSWORD', None)
        if not password:
            self.skipTest('No password provided for the test user.')

        self._login(
            'u1test+snapcraft@canonical.com', password,
            expect_success=True)

    def test_failed_login(self):
        self._login(
            'u1test+snapcraft@canonical.com',
            'wrongpassword',
            expect_success=False)
