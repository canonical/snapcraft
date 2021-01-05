# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2019 Canonical Ltd
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
from testtools.matchers import Equals

from snapcraft import storeapi

from . import FakeStoreCommandsBaseTestCase, get_sample_key


class CreateKeyTestCase(FakeStoreCommandsBaseTestCase):
    def setUp(self):
        super().setUp()

        self.fake_check_call = fixtures.MockPatch("subprocess.check_call")
        self.useFixture(self.fake_check_call)

    def test_create_key_already_exists(self):
        raised = self.assertRaises(
            storeapi.errors.KeyAlreadyRegisteredError, self.run_command, ["create-key"]
        )

        self.assertThat(
            str(raised), Equals("You have already registered a key named 'default'")
        )

    def test_create_key_already_registered(self):
        self.fake_store_account_info.mock.side_effect = [
            {
                "account_id": "abcd",
                "account_keys": [
                    {
                        "name": "new-key",
                        "public-key-sha3-384": (get_sample_key("default")["sha3-384"]),
                    }
                ],
            }
        ]
        raised = self.assertRaises(
            storeapi.errors.KeyAlreadyRegisteredError,
            self.run_command,
            ["create-key", "new-key"],
        )

        self.assertThat(
            str(raised), Equals("You have already registered a key named 'new-key'")
        )
        self.fake_check_call.mock.assert_not_called()

    def test_create_key(self):
        self.fake_store_account_info.mock.side_effect = [
            {
                "account_id": "abcd",
                "account_keys": [
                    {
                        "name": "old-key",
                        "public-key-sha3-384": (get_sample_key("default")["sha3-384"]),
                    }
                ],
            }
        ]

        result = self.run_command(["create-key", "new-key"])

        self.assertThat(result.exit_code, Equals(0))
