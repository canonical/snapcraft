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

import re
from unittest import mock

import fixtures
from simplejson.scanner import JSONDecodeError
from testtools.matchers import Contains, Equals, MatchesRegex, Not

from snapcraft import config, storeapi

from . import FakeStoreCommandsBaseTestCase


class LoginCommandTestCase(FakeStoreCommandsBaseTestCase):
    def test_login(self):
        # No 2fa
        self.fake_store_login.mock.side_effect = None

        result = self.run_command(["login"], input="user@example.com\nsecret\n")

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(storeapi.constants.TWO_FACTOR_WARNING))
        self.assertThat(result.output, Contains("Login successful."))
        self.fake_store_login.mock.assert_called_once_with(
            "user@example.com",
            mock.ANY,
            acls=None,
            packages=None,
            channels=None,
            expires=None,
            save=True,
            config_fd=None,
        )

    def test_login_with_2fa(self):
        self.fake_store_login.mock.side_effect = [
            storeapi.errors.StoreTwoFactorAuthenticationRequired(),
            None,
        ]

        # no exception raised.
        result = self.run_command(["login"], input="user@example.com\nsecret\n123456\n")

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output, Not(Contains(storeapi.constants.TWO_FACTOR_WARNING))
        )
        self.assertThat(result.output, Contains("Login successful."))

        self.assertThat(self.fake_store_login.mock.call_count, Equals(2))
        self.fake_store_login.mock.assert_has_calls(
            [
                mock.call(
                    "user@example.com",
                    "secret",
                    acls=None,
                    packages=None,
                    channels=None,
                    expires=None,
                    save=True,
                    config_fd=None,
                ),
                mock.call(
                    "user@example.com",
                    "secret",
                    one_time_password="123456",
                    acls=None,
                    packages=None,
                    channels=None,
                    expires=None,
                    save=True,
                    config_fd=None,
                ),
            ]
        )

    def test_successful_login_with(self):
        self.useFixture(
            fixtures.MockPatchObject(
                storeapi.StoreClient,
                "acl",
                return_value={
                    "snap_ids": None,
                    "channels": None,
                    "permissions": None,
                    "expires": "2018-01-01T00:00:00",
                },
            )
        )
        self.fake_store_login.mock.side_effect = None

        conf = config.Config()
        conf.set("macaroon", "test-macaroon")
        conf.set("unbound_discharge", "test-unbound-discharge")
        with open("exported-login", "w") as f:
            conf.save(config_fd=f)
            f.flush()

        result = self.run_command(["login", "--with", "exported-login"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("Login successful"))
        self.assertThat(
            result.output, MatchesRegex(r".*snaps:.*?No restriction", re.DOTALL)
        )
        self.assertThat(
            result.output, MatchesRegex(r".*channels:.*?No restriction", re.DOTALL)
        )
        self.assertThat(
            result.output, MatchesRegex(r".*permissions:.*?No restriction", re.DOTALL)
        )
        self.assertThat(
            result.output, MatchesRegex(r".*expires:.*?2018-01-01T00:00:00", re.DOTALL)
        )

        self.fake_store_login.mock.assert_called_once_with(
            "",
            "",
            acls=None,
            packages=None,
            channels=None,
            expires=None,
            save=True,
            config_fd=mock.ANY,
        )

    def test_login_failed_with_invalid_credentials(self):
        self.fake_store_login.mock.side_effect = storeapi.errors.InvalidCredentialsError(
            "error"
        )

        result = self.run_command(["login"], input="user@example.com\nbad-secret\n")

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(result.output, Contains(storeapi.constants.INVALID_CREDENTIALS))
        self.assertThat(result.output, Contains("Login failed."))

    def test_login_failed_with_store_authentication_error(self):
        self.fake_store_login.mock.side_effect = storeapi.errors.StoreAuthenticationError(
            "error"
        )

        raised = self.assertRaises(
            storeapi.errors.StoreAuthenticationError,
            self.run_command,
            ["login"],
            input="user@example.com\nbad-secret\n",
        )

        self.assertThat(raised.message, Equals("error"))

    def test_failed_login_with_store_account_info_error(self):
        response = mock.Mock()
        response.json.side_effect = JSONDecodeError("mock-fail", "doc", 1)
        response.status_code = 500
        response.reason = "Internal Server Error"
        self.fake_store_login.mock.side_effect = storeapi.errors.StoreAccountInformationError(
            response
        )

        result = self.run_command(["login"], input="user@example.com\nsecret\n\n")

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(
            result.output, Contains(storeapi.constants.ACCOUNT_INFORMATION_ERROR)
        )
        self.assertThat(result.output, Contains("Login failed."))

    def test_login_failed_with_dev_agreement_error(self):
        response = mock.Mock()
        response.status_code = 403
        response.reason = storeapi.constants.MISSING_AGREEMENT
        content = {
            "error_list": [
                {
                    "message": storeapi.constants.MISSING_AGREEMENT,
                    "extra": {"url": "http://fake-url.com", "api": "fake-api"},
                }
            ]
        }
        response.json.return_value = content
        self.fake_store_account_info.mock.side_effect = storeapi.errors.StoreAccountInformationError(
            response
        )

        result = self.run_command(["login"], input="user@example.com\nsecret\nn\n")

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(result.output, Contains(storeapi.constants.AGREEMENT_ERROR))
        self.assertThat(result.output, Contains("Login failed."))

    def test_failed_login_with_dev_namespace_error(self):
        response = mock.Mock()
        response.status_code = 403
        response.reason = storeapi.constants.MISSING_NAMESPACE
        content = {
            "error_list": [
                {
                    "message": storeapi.constants.MISSING_NAMESPACE,
                    "extra": {"url": "http://fake-url.com", "api": "fake-api"},
                }
            ]
        }
        response.json.return_value = content
        self.fake_store_account_info.mock.side_effect = storeapi.errors.StoreAccountInformationError(
            response
        )

        result = self.run_command(["login"], input="user@example.com\nsecret\n")

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(
            result.output,
            Contains(storeapi.constants.NAMESPACE_ERROR.format("http://fake-url.com")),
        )
        self.assertThat(result.output, Contains("Login failed."))

    def test_failed_login_with_unexpected_account_error(self):
        # Test to simulate get_account_info raising unexpected errors.
        response = mock.Mock()
        response.status_code = 500
        response.reason = "Internal Server Error"
        content = {
            "error_list": [
                {
                    "message": "Just another error",
                    "extra": {"url": "http://fake-url.com", "api": "fake-api"},
                }
            ]
        }
        response.json.return_value = content
        self.fake_store_account_info.mock.side_effect = storeapi.errors.StoreAccountInformationError(
            response
        )

        result = self.run_command(["login"], input="user@example.com\nsecret\n")

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(
            result.output, Contains(storeapi.constants.ACCOUNT_INFORMATION_ERROR)
        )
        self.assertThat(result.output, Contains("Login failed."))

    def test_failed_login_with_dev_agreement_error_with_choice(self):
        response = mock.Mock()
        response.status_code = 403
        response.reason = storeapi.constants.MISSING_AGREEMENT
        content = {
            "error_list": [
                {
                    "message": storeapi.constants.MISSING_AGREEMENT,
                    "extra": {"url": "http://fake-url.com", "api": "fake-api"},
                }
            ]
        }
        response.json.return_value = content
        self.fake_store_account_info.mock.side_effect = storeapi.errors.StoreAccountInformationError(
            response
        )

        result = self.run_command(["login"], input="user@example.com\nsecret\nn\n")

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(result.output, Contains(storeapi.constants.AGREEMENT_ERROR))
        self.assertThat(result.output, Contains("Login failed."))

    def test_failed_login_with_dev_agreement_error_with_choice_no(self):
        response = mock.Mock()
        response.status_code = 403
        response.reason = storeapi.constants.MISSING_AGREEMENT
        content = {
            "error_list": [
                {
                    "message": storeapi.constants.MISSING_AGREEMENT,
                    "extra": {"url": "http://fake-url.com", "api": "fake-api"},
                }
            ]
        }
        response.json.return_value = content
        self.fake_store_account_info.mock.side_effect = storeapi.errors.StoreAccountInformationError(
            response
        )

        result = self.run_command(["login"], input="user@example.com\nsecret\nn\n")

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(result.output, Contains(storeapi.constants.AGREEMENT_ERROR))
        self.assertThat(result.output, Contains("Login failed."))

    def test_failed_login_with_dev_agreement_error_with_choice_yes(self):
        response = mock.Mock()
        response.status_code = 403
        response.reason = storeapi.constants.MISSING_AGREEMENT
        content = {
            "error_list": [
                {
                    "message": storeapi.constants.MISSING_AGREEMENT,
                    "extra": {"url": "http://fake-url.com", "api": "fake-api"},
                }
            ]
        }
        response.json.return_value = content
        self.fake_store_account_info.mock.side_effect = storeapi.errors.StoreAccountInformationError(
            response
        )

        result = self.run_command(["login"], input="user@example.com\nsecret\ny\n")

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(
            result.output,
            Contains(
                storeapi.constants.AGREEMENT_SIGN_ERROR.format("http://fake-url.com")
            ),
        )
        self.assertThat(result.output, Contains("Login failed."))

    def test_failed_login_with_dev_agreement_error_with_sign_success(self):
        self.useFixture(
            fixtures.MockPatchObject(storeapi.StoreClient, "sign_developer_agreement")
        )
        response = mock.Mock()
        response.status_code = 403
        response.reason = storeapi.constants.MISSING_AGREEMENT
        content = {
            "error_list": [
                {
                    "message": storeapi.constants.MISSING_AGREEMENT,
                    "extra": {"url": "http://fake-url.com", "api": "fake-api"},
                }
            ]
        }
        response.json.return_value = content
        self.fake_store_account_info.mock.side_effect = storeapi.errors.StoreAccountInformationError(
            response
        )

        result = self.run_command(["login"], input="user@example.com\nsecret\ny\n")

        self.assertThat(result.exit_code, Equals(1))
        self.assertThat(
            result.output, Contains(storeapi.constants.ACCOUNT_INFORMATION_ERROR)
        )
