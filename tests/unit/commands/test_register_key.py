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

import json
from textwrap import dedent
from unittest import mock

import fixtures
from simplejson.scanner import JSONDecodeError
from testtools.matchers import Contains, Equals

from snapcraft import storeapi

from . import FakeStoreCommandsBaseTestCase, get_sample_key


class RegisterKeyTestCase(FakeStoreCommandsBaseTestCase):
    def test_register_key(self):
        result = self.run_command(
            ["register-key", "default"], input="user@example.com\nsecret\n"
        )

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("Registering key ..."))
        self.assertThat(
            result.output,
            Contains(
                'Done. The key "default" ({}) may be used to sign your '
                "assertions.".format(get_sample_key("default")["sha3-384"])
            ),
        )
        self.fake_store_login.mock.assert_called_once_with(
            email="user@example.com",
            password="secret",
            acls=["modify_account_key"],
            packages=None,
            channels=None,
            expires=None,
            save=False,
            config_fd=None,
        )
        self.fake_store_register_key.mock.call_once_with(
            dedent(
                """\
                type: account-key-request
                account-id: abcd
                name: default
                public-key-sha3-384: {}
                """
            ).format(get_sample_key("default")["sha3-384"])
        )

    def test_register_key_no_keys(self):
        self.fake_check_output.mock.side_effect = [json.dumps([]).encode()]

        raised = self.assertRaises(
            storeapi.errors.NoKeysError, self.run_command, ["register-key"]
        )

        self.assertThat(str(raised), Contains("You have no usable keys"))

    def test_register_key_no_keys_with_name(self):
        raised = self.assertRaises(
            storeapi.errors.NoSuchKeyError,
            self.run_command,
            ["register-key", "nonexistent"],
        )

        self.assertThat(
            str(raised), Contains("You have no usable key named 'nonexistent'")
        )

    def test_register_key_login_failed(self):
        self.fake_store_login.mock.side_effect = storeapi.http_clients.errors.InvalidCredentialsError(
            "error"
        )

        raised = self.assertRaises(
            storeapi.http_clients.errors.InvalidCredentialsError,
            self.run_command,
            ["register-key", "default"],
            input="user@example.com\nsecret\n",
        )

        assert (
            str(raised) == 'Invalid credentials: error. Have you run "snapcraft login"?'
        )

    def test_register_key_account_info_failed(self):
        response = mock.Mock()
        response.json.side_effect = JSONDecodeError("mock-fail", "doc", 1)
        response.status_code = 500
        response.reason = "Internal Server Error"
        self.fake_store_account_info.mock.side_effect = storeapi.errors.StoreAccountInformationError(
            response
        )

        # Fake the login check
        self.useFixture(fixtures.MockPatch("snapcraft._store.login", return_value=True))

        raised = self.assertRaises(
            storeapi.errors.StoreAccountInformationError,
            self.run_command,
            ["register-key", "default"],
        )

        self.assertThat(
            str(raised),
            Equals(
                "Error fetching account information from store: "
                "500 Internal Server Error"
            ),
        )

    def test_register_key_failed(self):
        response = mock.Mock()
        response.json.side_effect = JSONDecodeError("mock-fail", "doc", 1)
        response.status_code = 500
        response.reason = "Internal Server Error"
        self.fake_store_register_key.mock.side_effect = storeapi.errors.StoreKeyRegistrationError(
            response
        )

        raised = self.assertRaises(
            storeapi.errors.StoreKeyRegistrationError,
            self.run_command,
            ["register-key", "default"],
            input="user@example.com\nsecret\n",
        )

        self.assertThat(
            str(raised), Equals("Key registration failed: 500 Internal Server Error")
        )

    def test_register_key_select_key(self):
        result = self.run_command(
            ["register-key"], input="x\n2\nuser@example.com\nsecret\n"
        )

        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Select a key:

              Number  Name     SHA3-384 fingerprint
                   1  default  {default_sha3_384}
                   2  another  {another_sha3_384}
            """
                ).format(
                    default_sha3_384=get_sample_key("default")["sha3-384"],
                    another_sha3_384=get_sample_key("another")["sha3-384"],
                )
            ),
        )
        self.assertThat(
            self.fake_logger.output,
            Contains(
                'Done. The key "another" ({}) may be used to sign your '
                "assertions.\n".format(get_sample_key("another")["sha3-384"])
            ),
        )

        self.fake_store_register_key.mock.assert_called_once_with(
            dedent(
                """\
            type: account-key-request
            account-id: abcd
            name: another
            public-key-sha3-384: {}
            """
            ).format(get_sample_key("another")["sha3-384"])
        )
