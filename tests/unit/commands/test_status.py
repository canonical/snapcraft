# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2019 Canonical Ltd
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
from . import FakeStoreCommandsBaseTestCase


class StatusCommandTestCase(FakeStoreCommandsBaseTestCase):
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

    def test_status_without_login_must_ask(self):
        self.fake_store_account_info.mock.side_effect = [
            storeapi.errors.InvalidCredentialsError("error"),
            {"snaps": {"16": {"snap-test": {"snap-id": "snap-test-snap-id"}}}},
            {"snaps": {"16": {"snap-test": {"snap-id": "snap-test-snap-id"}}}},
            {"snaps": {"16": {"snap-test": {"snap-id": "snap-test-snap-id"}}}},
        ]
        self.fake_store_status.mock.return_value = {
            "channel_map_tree": {"latest": {"16": self.expected}}
        }

        result = self.run_command(
            ["status", "snap-test"], input="user@example.com\nsecret\n"
        )
        self.assertThat(
            result.output, Contains("You are required to login before continuing.")
        )

    def test_status_with_3rd_party_snap(self):
        raised = self.assertRaises(
            storeapi.errors.SnapNotFoundError, self.run_command, ["status", "snap-test"]
        )

        self.assertThat(
            str(raised), Equals("Snap 'snap-test' was not found in '16' series.")
        )

    def test_status_with_3rd_party_snap_by_arch(self):
        raised = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.run_command,
            ["status", "snap-test", "--arch=arm64"],
        )

        self.assertThat(
            str(raised),
            Equals("Snap 'snap-test' for 'arm64' was not found in '16' series."),
        )

    def test_status_with_3rd_party_snap_by_series(self):
        raised = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.run_command,
            ["status", "snap-test", "--series=18"],
        )

        self.assertThat(
            str(raised), Equals("Snap 'snap-test' was not found in '18' series.")
        )

    def test_status_by_unknown_arch(self):
        raised = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.run_command,
            ["status", "snap-test", "--arch=some-arch"],
        )

        self.assertThat(
            str(raised),
            Equals("Snap 'snap-test' for 'some-arch' was not found in '16' series."),
        )

    def test_status_by_unknown_series(self):
        raised = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            self.run_command,
            ["status", "snap-test", "--series=some-series"],
        )

        self.assertThat(
            str(raised),
            Equals("Snap 'snap-test' was not found in 'some-series' series."),
        )

    def test_status(self):
        self.fake_store_status.mock.return_value = {
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
        self.fake_store_status.mock.assert_called_once_with(
            "snap-test-snap-id", "16", None
        )

    def test_status_by_arch(self):
        self.fake_store_status.mock.return_value = {
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
        self.fake_store_status.mock.assert_called_once_with(
            "snap-test-snap-id", "16", "i386"
        )

    def test_status_by_series(self):
        self.fake_store_status.mock.return_value = {
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
        self.fake_store_status.mock.assert_called_once_with(
            "snap-test-snap-id", "16", None
        )

    def test_status_including_branch(self):
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
        self.fake_store_status.mock.return_value = {
            "channel_map_tree": {"latest": {"16": expected}}
        }

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
        )
        self.fake_store_status.mock.assert_called_once_with(
            "snap-test-snap-id", "16", None
        )
