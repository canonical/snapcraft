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
from testtools.matchers import Contains, Equals, MatchesRegex

from snapcraft_legacy import storeapi

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
            ttl=mock.ANY,
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
            ttl=mock.ANY,
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
            ["export-login", "--expires=2018-02-01T00:00:00Z", "exported"],
            input="user@example.com\nsecret\n",
        )
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
        self.assertThat(result.exit_code, Equals(0))

        self.fake_store_login.mock.assert_called_once_with(
            email="user@example.com",
            password=mock.ANY,
            acls=None,
            packages=None,
            channels=None,
            ttl=mock.ANY,
        )

    def test_bad_date_format(self):
        result = self.run_command(
            ["export-login", "--expires=20180201", "exported"],
            input="user@example.com\nsecret\n",
        )
        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(
            result.output,
            Contains(
                "Error: Invalid value: The expiry follow an ISO 8601 format "
                "('%Y-%m-%d' or '%Y-%m-%dT%H:%M:%SZ')"
            ),
        )
