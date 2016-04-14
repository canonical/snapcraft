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

import uuid
import fixtures


import store_tests


class LoginLogoutTestCase(store_tests.TestCase):

    def setUp(self):
        super().setUp()
        # FIXME: Switch to macaroons as soon as they are available for
        # register-name -- vila 2016-04-14
        if False:
            self.useFixture(
                fixtures.EnvironmentVariable('SNAPCRAFT_WITH_MACAROONS', '1'))
        else:
            self.useFixture(
                fixtures.EnvironmentVariable('SNAPCRAFT_WITH_MACAROONS', None))

    # FIXME: The store doesn't provide a way to unregister a name *and*
    # registrations are rate-limited for a given user. We work around that by
    # creating unique names and assuming we only run against staging or local
    # dev instances -- vila 2016-04-08
    def test_successful_register(self):
        self.addCleanup(self.logout)
        self.login()
        uniq_name = 'delete-me-{}'.format(str(uuid.uuid4().int)[:32])
        response = self.register(uniq_name)
        self.assertTrue(response.ok)
        # We get a snap_id back
        self.assertIn('snap_id', response.json())

    def test_failed_login(self):
        self.addCleanup(self.logout)
        self.login(password='wrongpassword')
        uniq_name = 'delete-me-{}'.format(str(uuid.uuid4().int)[:32])
        response = self.register(uniq_name)
        self.assertEqual(401, response.status_code)
