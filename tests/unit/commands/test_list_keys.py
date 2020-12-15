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

from textwrap import dedent

from testtools.matchers import Contains, Equals

from snapcraft import storeapi

from . import FakeStoreCommandsBaseTestCase, get_sample_key


class ListKeysCommandTestCase(FakeStoreCommandsBaseTestCase):

    command_name = "list-keys"

    def test_list_keys_snapd_not_installed(self):
        self.fake_package_installed.mock.return_value = False

        raised = self.assertRaises(
            storeapi.errors.MissingSnapdError, self.run_command, [self.command_name]
        )

        self.assertThat(str(raised), Contains("The snapd package is not installed."))
        self.assertThat(str(raised), Contains("The snapd package is not installed."))
        self.fake_package_installed.mock.assert_called_with("snapd")
        self.fake_check_output.mock.assert_not_called()

    def test_command_without_login_must_ask(self):
        # TODO: look into why this many calls are done inside snapcraft.storeapi
        self.fake_store_account_info.mock.side_effect = [
            storeapi.errors.InvalidCredentialsError("error"),
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
            Contains(
                dedent(
                    """\
                Name     SHA3-384 fingerprint
            *   default  {default_sha3_384}
            -   another  {another_sha3_384}  (not registered)
            """
                ).format(
                    default_sha3_384=get_sample_key("default")["sha3-384"],
                    another_sha3_384=get_sample_key("another")["sha3-384"],
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
            Contains(
                "No keys have been registered. "
                "See 'snapcraft register-key --help' to register a key."
            ),
        )
