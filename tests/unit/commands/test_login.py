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

import pathlib
import re
from unittest import mock

import fixtures
import pytest
from simplejson.scanner import JSONDecodeError
from testtools.matchers import Contains, Equals, MatchesRegex, Not

from snapcraft import storeapi

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
            email="user@example.com",
            password=mock.ANY,
            acls=None,
            packages=None,
            channels=None,
            expires=None,
            save=True,
            config_fd=None,
        )

    def test_login_with_2fa(self):
        self.fake_store_login.mock.side_effect = [
            storeapi.http_clients.errors.StoreTwoFactorAuthenticationRequired(),
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
                    email="user@example.com",
                    password="secret",
                    acls=None,
                    packages=None,
                    channels=None,
                    expires=None,
                    save=True,
                    config_fd=None,
                ),
                mock.call(
                    email="user@example.com",
                    password="secret",
                    otp="123456",
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

        pathlib.Path("exported-login").touch()

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
            email="",
            password="",
            acls=None,
            packages=None,
            channels=None,
            expires=None,
            save=True,
            config_fd=mock.ANY,
        )

    def test_login_failed_with_invalid_credentials(self):
        self.fake_store_login.mock.side_effect = storeapi.http_clients.errors.InvalidCredentialsError(
            "error"
        )

        with pytest.raises(
            storeapi.http_clients.errors.InvalidCredentialsError
        ) as exc_info:
            self.run_command(["login"], input="user@example.com\nbadsecret\n")

        assert (
            str(exc_info.value)
            == 'Invalid credentials: error. Have you run "snapcraft login"?'
        )

    def test_login_failed_with_store_authentication_error(self):
        self.fake_store_login.mock.side_effect = storeapi.http_clients.errors.StoreAuthenticationError(
            "error"
        )

        raised = self.assertRaises(
            storeapi.http_clients.errors.StoreAuthenticationError,
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

        with pytest.raises(storeapi.errors.StoreAccountInformationError) as exc_info:
            self.run_command(["login"], input="user@example.com\nsecret\n\n")

        assert (
            str(exc_info.value)
            == "Error fetching account information from store: 500 Internal Server Error"
        )

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

        with pytest.raises(storeapi.errors.NeedTermsSignedError) as exc_info:
            self.run_command(["login"], input="user@example.com\nsecret\n")

        assert (
            str(exc_info.value)
            == "Developer Terms of Service agreement must be signed before continuing: You need to set a username. It will appear in the developer field alongside the other details for your snap. Please visit http://fake-url.com and login again."
        )

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

        with pytest.raises(storeapi.errors.StoreAccountInformationError) as exc_info:
            self.run_command(["login"], input="user@example.com\nsecret\n\n")

        assert (
            str(exc_info.value)
            == "Error fetching account information from store: Just another error"
        )

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

        with pytest.raises(storeapi.errors.NeedTermsSignedError) as exc_info:
            self.run_command(["login"], input="user@example.com\nsecret\nn\n")

        assert (
            str(exc_info.value)
            == "Developer Terms of Service agreement must be signed before continuing: You must agree to the developer terms and conditions to upload snaps."
        )

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

        with pytest.raises(storeapi.errors.NeedTermsSignedError) as exc_info:
            self.run_command(["login"], input="user@example.com\nsecret\ny\n")

        assert (
            str(exc_info.value)
            == "Developer Terms of Service agreement must be signed before continuing: Unexpected error encountered during signing the developer terms and conditions. Please visit http://fake-url.com and agree to the terms and conditions before continuing."
        )
