# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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
import logging
from textwrap import dedent
from unittest import mock

import fixtures
from testtools.matchers import Contains, Equals

from snapcraft import storeapi
from tests import fixture_setup
from . import CommandBaseTestCase


class RevisionsCommandBaseTestCase(CommandBaseTestCase):
    def setUp(self):
        super().setUp()
        self.expected = [
            {
                "series": ["16"],
                "channels": [],
                "version": "2.0.1",
                "timestamp": "2016-09-27T19:23:40Z",
                "current_channels": ["beta", "edge"],
                "arch": "i386",
                "revision": 2,
            },
            {
                "series": ["16"],
                "channels": ["stable", "edge"],
                "version": "2.0.2",
                "timestamp": "2016-09-27T18:38:43Z",
                "current_channels": ["stable", "candidate", "beta"],
                "arch": "amd64",
                "revision": 1,
            },
        ]


class RevisionsCommandTestCase(RevisionsCommandBaseTestCase):

    scenarios = [
        ("list-revisions", dict(command_name="list-revisions")),
        ("revisions", dict(command_name="revisions")),
    ]

    def test_revisions_without_snap_raises_exception(self):
        result = self.run_command([self.command_name])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(result.output, Contains("Usage:"))

    def test_revisions_with_no_permissions(self):
        raised = self.assertRaises(
            storeapi.errors.InvalidCredentialsError,
            self.run_command,
            [self.command_name, "snap-test"],
        )

        self.assertThat(str(raised), Contains("Invalid credentials"))

    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    def test_revisions_with_3rd_party_snap(self, mock_account_api):
        mock_account_api.return_value = {}

        raised = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.run_command,
            [self.command_name, "snap-test"],
        )

        self.assertThat(
            str(raised), Equals("Snap 'snap-test' was not found in '16' series.")
        )

    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    def test_revisions_with_3rd_party_snap_by_arch(self, mock_account_api):
        mock_account_api.return_value = {}

        raised = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.run_command,
            [self.command_name, "snap-test", "--arch=arm64"],
        )

        self.assertThat(
            str(raised),
            Equals("Snap 'snap-test' for 'arm64' was not found in '16' series."),
        )

    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    def test_revisions_with_3rd_party_snap_by_series(self, mock_account_api):
        mock_account_api.return_value = {}

        raised = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.run_command,
            [self.command_name, "snap-test", "--series=18"],
        )

        self.assertThat(
            str(raised), Equals("Snap 'snap-test' was not found in '18' series.")
        )

    @mock.patch.object(storeapi._sca_client.SCAClient, "snap_revisions")
    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    def test_revisions_by_unknown_arch(self, mock_account_api, mock_revisions):
        mock_revisions.return_value = {}

        raised = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.run_command,
            [self.command_name, "snap-test", "--arch=some-arch"],
        )

        self.assertThat(
            str(raised),
            Equals("Snap 'snap-test' for 'some-arch' was not found in '16' series."),
        )

    @mock.patch.object(storeapi._sca_client.SCAClient, "snap_revisions")
    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    def test_revisions_by_unknown_series(self, mock_account_api, mock_revisions):
        mock_revisions.return_value = {}

        raised = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.run_command,
            [self.command_name, "snap-test", "--series=some-series"],
        )

        self.assertThat(
            str(raised),
            Equals("Snap 'snap-test' was not found in 'some-series' series."),
        )

    @mock.patch.object(storeapi.StoreClient, "get_snap_revisions")
    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    def test_revisions(self, mock_account_api, mock_revisions):
        mock_revisions.return_value = self.expected

        result = self.run_command([self.command_name, "snap-test"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Rev.    Uploaded              Arch    Version    Channels
            2       2016-09-27T19:23:40Z  i386    2.0.1      -
            1       2016-09-27T18:38:43Z  amd64   2.0.2      stable*, edge"""
                )
            ),
        )  # noqa
        mock_revisions.assert_called_once_with("snap-test", "16", None)

    @mock.patch.object(storeapi.StoreClient, "get_snap_revisions")
    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    def test_revisions_by_arch(self, mock_account_api, mock_revisions):
        fake_terminal = fixture_setup.FakeTerminal()
        self.useFixture(fake_terminal)

        mock_revisions.return_value = [
            rev for rev in self.expected if rev["arch"] == "amd64"
        ]

        result = self.run_command([self.command_name, "snap-test", "--arch=amd64"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Rev.    Uploaded              Arch    Version    Channels
            1       2016-09-27T18:38:43Z  amd64   2.0.2      stable*, edge"""
                )
            ),
        )  # noqa
        mock_revisions.assert_called_once_with("snap-test", "16", "amd64")

    @mock.patch.object(storeapi.StoreClient, "get_snap_revisions")
    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    def test_revisions_by_series(self, mock_account_api, mock_revisions):
        mock_revisions.return_value = self.expected

        result = self.run_command([self.command_name, "snap-test", "--series=16"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Rev.    Uploaded              Arch    Version    Channels
            2       2016-09-27T19:23:40Z  i386    2.0.1      -
            1       2016-09-27T18:38:43Z  amd64   2.0.2      stable*, edge"""
                )
            ),
        )  # noqa
        mock_revisions.assert_called_once_with("snap-test", "16", None)


class DeprecatedHistoryCommandTestCase(RevisionsCommandBaseTestCase):
    @mock.patch.object(storeapi.StoreClient, "get_snap_revisions")
    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    def test_history_with_deprecation_message(self, mock_account_api, mock_revisions):
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        result = self.run_command(["history", "snap-test"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            self.fake_logger.output,
            Contains(
                "DEPRECATED: The 'history' command has been replaced by "
                "'list-revisions'."
            ),
        )
