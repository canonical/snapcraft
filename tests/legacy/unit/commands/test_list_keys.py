# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2023 Canonical Ltd
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

from textwrap import dedent

import fixtures
from testtools.matchers import Contains, Equals

from . import FAKE_UNAUTHORIZED_ERROR, FakeStoreCommandsBaseTestCase, get_sample_key


class ListKeysCommandTestCase(FakeStoreCommandsBaseTestCase):
    command_name = "list-keys"

    def test_command_without_login_must_ask(self):
        # TODO: look into why this many calls are done inside snapcraft_legacy.storeapi
        self.fake_store_account_info.mock.side_effect = [
            FAKE_UNAUTHORIZED_ERROR,
            {"account_id": "abcd", "account_keys": list()},
            {"account_id": "abcd", "account_keys": list()},
            {"account_id": "abcd", "account_keys": list()},
        ]

        result = self.run_command(
            [self.command_name], input="user@example.com\nsecret\n"
        )
        self.assertThat(
            result.output, Contains("You are required to login before continuing.")
        )

    def test_list_keys_successfully(self):
        self.fake_store_account_info.mock.return_value = {
            "account_id": "abcd",
            "account_keys": [
                {
                    "name": "default",
                    "public-key-sha3-384": (get_sample_key("default")["sha3-384"]),
                }
            ],
        }

        result = self.run_command(
            [self.command_name], input="user@example.com\nsecret\n"
        )

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                dedent(
                    f"""\
                    The following keys are available on this system:
                        Name     SHA3-384 fingerprint
                    *   default  {get_sample_key("default")["sha3-384"]}
                    -   another  {get_sample_key("another")["sha3-384"]}  (not registered)
                    """
                )
            ),
        )

    def test_list_keys_no_keys_on_system(self):
        self.fake_store_account_info.mock.return_value = {
            "account_id": "abcd",
            "account_keys": [
                {
                    "name": "default",
                    "public-key-sha3-384": (get_sample_key("default")["sha3-384"]),
                }
            ],
        }

        self.useFixture(
            fixtures.MockPatch("subprocess.check_output", return_value="[]".encode())
        )

        result = self.run_command(
            [self.command_name], input="user@example.com\nsecret\n"
        )

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                dedent(
                    """\
                    No keys have been created on this system. See 'snapcraft create-key --help' to create a key.
                    The following SHA3-384 key fingerprints have been registered but are not available on this system:
                    - vdEeQvRxmZ26npJCFaGnl-VfGz0lU2jZZkWp_s7E-RxVCNtH2_mtjcxq2NkDKkIp
            """
                )
            ),
        )

    def test_alias(self):
        self.command_name = "keys"
        self.fake_store_account_info.mock.return_value = {
            "account_id": "abcd",
            "account_keys": [
                {
                    "name": "default",
                    "public-key-sha3-384": (get_sample_key("default")["sha3-384"]),
                }
            ],
        }

        result = self.run_command(
            [self.command_name], input="user@example.com\nsecret\n"
        )

        self.assertThat(result.exit_code, Equals(0))

    def test_list_keys_without_registered(self):
        result = self.run_command(
            [self.command_name], input="user@example.com\nsecret\n"
        )

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                dedent(
                    f"""\
                    The following keys are available on this system:
                        Name     SHA3-384 fingerprint
                    -   default  {get_sample_key("default")["sha3-384"]}  (not registered)
                    -   another  {get_sample_key("another")["sha3-384"]}  (not registered)
                    No keys have been registered with this account. See 'snapcraft register-key --help' to register a key.
                    """
                )
            ),
        )
