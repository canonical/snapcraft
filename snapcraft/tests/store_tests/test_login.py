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

import testscenarios

from snapcraft import config
from snapcraft.tests import store_tests


load_tests = testscenarios.load_tests_apply_scenarios


class TestLoginLogout(store_tests.RecordedTestCase):

    def test_successful_login(self):
        self.addCleanup(self.logout)
        res = self.login()
        self.assertTrue(res['success'])
        # Credentials have been saved
        self.assertTrue(os.path.exists(config.Config.save_path()))
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


class TestLoginErrors(store_tests.TestCase):

    def test_get_macarron_error(self):
        def cookie(url, data, headers):
            class Response(object):
                ok = False
                text = 'No macaroon, want a cookie ?'
            return Response
        self.store.post = cookie
        response = self.login('email@example.com', 'secret')
        self.assertEqual('No macaroon, want a cookie ?', response['errors'])
