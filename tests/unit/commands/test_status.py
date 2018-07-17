# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2017 Canonical Ltd
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
from unittest import mock

from testtools.matchers import Contains, Equals

from snapcraft import storeapi
from . import CommandBaseTestCase


class StatusCommandTestCase(CommandBaseTestCase):
    def setUp(self):
        super().setUp()
        self.expected = {
            "i386": [
                {"info": "none", "channel": "stable"},
                {"info": "none", "channel": "beta"},
                {
                    "info": "specific",
                    "version": "1.0-i386",
                    "channel": "edge",
                    "revision": 3,
                },
            ],
            "amd64": [
                {
                    "info": "specific",
                    "version": "1.0-amd64",
                    "channel": "stable",
                    "revision": 2,
                },
                {
                    "info": "specific",
                    "version": "1.1-amd64",
                    "channel": "beta",
                    "revision": 4,
                },
                {"info": "tracking", "channel": "edge"},
            ],
        }

    def test_status_without_snap_raises_exception(self):
        result = self.run_command(["status"])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(result.output, Contains("Usage:"))

    def test_status_with_no_permissions(self):
        raised = self.assertRaises(
            storeapi.errors.InvalidCredentialsError,
            self.run_command,
            ["status", "snap-test"],
        )

        self.assertThat(str(raised), Contains("Invalid credentials"))

    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    def test_status_with_3rd_party_snap(self, mock_account_api):
        mock_account_api.return_value = {}

        raised = self.assertRaises(
            storeapi.errors.SnapNotFoundError, self.run_command, ["status", "snap-test"]
        )

        self.assertThat(
            str(raised), Equals("Snap 'snap-test' was not found in '16' series.")
        )

    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    def test_status_with_3rd_party_snap_by_arch(self, mock_account_api):
        mock_account_api.return_value = {}

        raised = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.run_command,
            ["status", "snap-test", "--arch=arm64"],
        )

        self.assertThat(
            str(raised),
            Equals("Snap 'snap-test' for 'arm64' was not found in '16' series."),
        )

    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    def test_status_with_3rd_party_snap_by_series(self, mock_account_api):
        mock_account_api.return_value = {}

        raised = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.run_command,
            ["status", "snap-test", "--series=18"],
        )

        self.assertThat(
            str(raised), Equals("Snap 'snap-test' was not found in '18' series.")
        )

    @mock.patch.object(storeapi._sca_client.SCAClient, "snap_status")
    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    def test_status_by_unknown_arch(self, mock_account_api, mock_status):
        mock_status.return_value = {}

        raised = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.run_command,
            ["status", "snap-test", "--arch=some-arch"],
        )

        self.assertThat(
            str(raised),
            Equals("Snap 'snap-test' for 'some-arch' was not found in '16' series."),
        )

    @mock.patch.object(storeapi._sca_client.SCAClient, "snap_status")
    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    def test_status_by_unknown_series(self, mock_account_api, mock_status):
        mock_status.return_value = {}

        mock_status.return_value = {}

        raised = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.run_command,
            ["status", "snap-test", "--series=some-series"],
        )

        self.assertThat(
            str(raised),
            Equals("Snap 'snap-test' was not found in 'some-series' series."),
        )

    @mock.patch.object(storeapi.StoreClient, "get_snap_status")
    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    def test_status(self, mock_account_api, mock_status):
        mock_status.return_value = {
            "channel_map_tree": {"latest": {"16": self.expected}}
        }

        result = self.run_command(["status", "snap-test"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Track    Arch    Channel    Version    Revision
            latest   amd64   stable     1.0-amd64  2
                             beta       1.1-amd64  4
                             edge       ^          ^
                     i386    stable     -          -
                             beta       -          -
                             edge       1.0-i386   3"""
                )
            ),
        )
        mock_status.assert_called_once_with("snap-test", "16", None)

    @mock.patch.object(storeapi.StoreClient, "get_snap_status")
    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    def test_status_by_arch(self, mock_account_api, mock_status):
        mock_status.return_value = {
            "channel_map_tree": {"latest": {"16": {"i386": self.expected["i386"]}}}
        }

        result = self.run_command(["status", "snap-test", "--arch=i386"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Track    Arch    Channel    Version    Revision
            latest   i386    stable     -          -
                             beta       -          -
                             edge       1.0-i386   3"""
                )
            ),
        )
        mock_status.assert_called_once_with("snap-test", "16", "i386")

    @mock.patch.object(storeapi.StoreClient, "get_snap_status")
    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    def test_status_by_series(self, mock_account_api, mock_status):
        mock_status.return_value = {
            "channel_map_tree": {"latest": {"16": self.expected}}
        }

        result = self.run_command(["status", "snap-test", "--series=16"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Track    Arch    Channel    Version    Revision
            latest   amd64   stable     1.0-amd64  2
                             beta       1.1-amd64  4
                             edge       ^          ^
                     i386    stable     -          -
                             beta       -          -
                             edge       1.0-i386   3"""
                )
            ),
        )
        mock_status.assert_called_once_with("snap-test", "16", None)

    @mock.patch.object(storeapi.StoreClient, "get_snap_status")
    @mock.patch.object(storeapi.StoreClient, "get_account_information")
    def test_status_including_branch(self, mock_account_api, mock_status):
        expected = {
            "i386": [
                {"info": "none", "channel": "stable"},
                {"info": "none", "channel": "beta"},
                {
                    "info": "branch",
                    "version": "1.0-i386",
                    "channel": "stable/hotfix",
                    "revision": 3,
                    "expires_at": "2017-05-21T18:52:14.578435",
                },
            ]
        }
        mock_status.return_value = {"channel_map_tree": {"latest": {"16": expected}}}

        result = self.run_command(["status", "snap-test"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Contains(
                dedent(
                    """\
            Track    Arch    Channel        Version    Revision    Expires at
            latest   i386    stable         -          -
                             beta           -          -
                             stable/hotfix  1.0-i386   3           2017-05-21T18:52:14.578435
            """
                )
            ),
        )  # noqa
        mock_status.assert_called_once_with("snap-test", "16", None)
