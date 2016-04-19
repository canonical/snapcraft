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
import testscenarios

from snapcraft import config

import store_tests


load_tests = testscenarios.load_tests_apply_scenarios


class TestLoginLogout(store_tests.TestCase):

    scenarios = (('OAuth', dict(with_macaroons=False)),
                 ('macaroons', dict(with_macaroons=True)))

    def setUp(self):
        super().setUp()
        if self.with_macaroons:
            self.useFixture(
                fixtures.EnvironmentVariable('SNAPCRAFT_WITH_MACAROONS', '1'))
        else:
            self.useFixture(
                fixtures.EnvironmentVariable('SNAPCRAFT_WITH_MACAROONS', None))

    def test_successful_login(self):
        self.addCleanup(self.logout)
        res = self.login()
        self.assertTrue(res['success'])
        # Credentials have been saved
        self.assertTrue(os.path.exists(config.Config.save_path()))
        if not self.with_macaroons:
            return
        conf = self.store.conf
        self.assertIsNotNone(conf.get('store_read'))
        self.assertIsNotNone(conf.get('store_write'))
        self.assertIsNotNone(conf.get('package_access'))
        self.assertIsNotNone(conf.get('package_upload'))

    def test_failed_login(self):
        res = self.login(password='wrongpassword')
        self.assertFalse(res['success'])
        # Credentials have not been saved
        self.assertFalse(os.path.exists(config.Config.save_path()))

    def test_logout_clear_config(self):
        self.addCleanup(self.logout)
        res = self.login()
        self.assertTrue(res['success'])
        res = self.logout()
        conf = config.Config()
        self.assertTrue(conf.is_empty())
