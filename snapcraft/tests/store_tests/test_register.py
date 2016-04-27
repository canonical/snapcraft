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


from snapcraft.tests import store_tests


class RegisterTestCase(store_tests.TestCase):

    def setUp(self):
        super().setUp()
        # Always login, the command fails if the required credentials are not
        # available (see integration_tests/test_register.py)
        self.addCleanup(self.logout)
        self.login()

    # FIXME: The store doesn't provide a way to unregister a name *and*
    # registrations are rate-limited for a given user. We work around that by
    # creating unique names and assuming we only run against staging or local
    # dev instances -- vila 2016-04-08
    def test_successful_register(self):
        uniq_name = 'delete-me-{}'.format(str(uuid.uuid4().int)[:32])
        response = self.register(uniq_name)
        # A single user can't register too often, run less tests :-/
        if response.status_code == 400:
            json_resp = response.json()
            all_msg = json_resp['__all__']
            if all_msg[0].startswith('You must wait'):
                self.skipTest('Register hits rate limitation.')
        self.assertTrue(response.ok)
        # We get a snap_id back
        self.assertIn('snap_id', response.json())
