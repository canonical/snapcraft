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

import fixtures

import store_tests


class LoginLogoutTestCase(store_tests.TestCase):

    def setUp(self):
        super().setUp()
        self.useFixture(
            fixtures.EnvironmentVariable('SNAPCRAFT_WITH_MACAROONS', None))

    def test_successful_login(self):
        self.addCleanup(self.logout)
        res = self.login()
        self.assertTrue(res['success'])

    def test_failed_login(self):
        res = self.login(password='wrongpassword')
        self.assertFalse(res['success'])


class LoginLogoutWithMacaroonsTestCase(store_tests.TestCase):

    def setUp(self):
        super().setUp()
        self.useFixture(
            fixtures.EnvironmentVariable('SNAPCRAFT_WITH_MACAROONS', '1'))

    def test_successful_login(self):
        self.addCleanup(self.logout)
        res = self.login()
        self.assertTrue(res['success'])

    def test_failed_login(self):
        res = self.login(password='wrongpassword')
        self.assertFalse(res['success'])
