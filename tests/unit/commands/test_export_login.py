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
import pytest
from testtools.matchers import Contains, Equals, MatchesRegex, Not

from snapcraft import storeapi

from . import FakeStoreCommandsBaseTestCase


class ExportLoginCommandTestCase(FakeStoreCommandsBaseTestCase):
    def setUp(self):
        super().setUp()

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

    def test_successful_export(self):
        result = self.run_command(
            ["export-login", "exported"], input="user@example.com\nsecret\n"
        )

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(storeapi.constants.TWO_FACTOR_WARNING))
        self.assertThat(result.output, Contains("Login successfully exported"))
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
            email="user@example.com",
            password="secret",
            acls=None,
            packages=None,
            channels=None,
            expires=None,
            save=False,
            config_fd=None,
        )

    def test_successful_export_stdout(self):
        result = self.run_command(
            ["export-login", "-"], input="user@example.com\nsecret\n"
        )

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(storeapi.constants.TWO_FACTOR_WARNING))
        self.assertThat(result.output, Contains("Exported login starts on next line"))
        self.assertThat(
            result.output, Contains("Login successfully exported and printed above")
        )
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
            email="user@example.com",
            password="secret",
            acls=None,
            packages=None,
            channels=None,
            expires=None,
            save=False,
            config_fd=None,
        )

    def test_successful_export_expires(self):
        self.useFixture(
            fixtures.MockPatchObject(
                storeapi.StoreClient,
                "acl",
                return_value={
                    "snap_ids": None,
                    "channels": None,
                    "permissions": None,
                    "expires": "2018-02-01T00:00:00",
                },
            )
        )

        result = self.run_command(
            ["export-login", "--expires=2018-02-01T00:00:00", "exported"],
            input="user@example.com\nsecret\n",
        )
        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains(storeapi.constants.TWO_FACTOR_WARNING))
        self.assertThat(result.output, Contains("Login successfully exported"))
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
            result.output, MatchesRegex(r".*expires:.*?2018-02-01T00:00:00", re.DOTALL)
        )

        self.fake_store_login.mock.assert_called_once_with(
            email="user@example.com",
            password=mock.ANY,
            acls=None,
            packages=None,
            channels=None,
            expires="2018-02-01T00:00:00",
            save=False,
            config_fd=None,
        )

    def test_successful_login_with_2fa(self):
        self.fake_store_login.mock.side_effect = [
            storeapi.http_clients.errors.StoreTwoFactorAuthenticationRequired(),
            None,
        ]

        result = self.run_command(
            ["export-login", "exported"], input="user@example.com\nsecret\n123456"
        )

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output, Not(Contains(storeapi.constants.TWO_FACTOR_WARNING))
        )
        self.assertThat(result.output, Contains("Login successfully exported"))
        self.assertThat(
            result.output, MatchesRegex(r".*snaps:.*?No restriction", re.DOTALL)
        )
        self.assertThat(
            result.output, MatchesRegex(r".*channels:.*?['edge']", re.DOTALL)
        )
        self.assertThat(
            result.output, MatchesRegex(r".*permissions:.*?No restriction", re.DOTALL)
        )
        self.assertThat(
            result.output, MatchesRegex(r".*expires:.*?2018-01-01T00:00:00", re.DOTALL)
        )

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
                    save=False,
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
                    save=False,
                    config_fd=None,
                ),
            ]
        )

    def test_failed_login_with_invalid_credentials(self):
        self.fake_store_login.mock.side_effect = storeapi.http_clients.errors.InvalidCredentialsError(
            "error"
        )

        with pytest.raises(
            storeapi.http_clients.errors.InvalidCredentialsError
        ) as exc_info:
            self.run_command(
                ["export-login", "exported"],
                input="bad-user@example.com\nbad-password\n",
            )

        assert (
            str(exc_info.value)
            == 'Invalid credentials: error. Have you run "snapcraft login"?'
        )
