# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2020 Canonical Ltd
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
from snapcraft.storeapi.v2.snap_channel_map import SnapChannelMap
from . import FakeStoreCommandsBaseTestCase


class StatusCommandTestCase(FakeStoreCommandsBaseTestCase):
    def setUp(self):
        super().setUp()
        self.channel_map_payload = {
            "channel-map": [
                {
                    "architecture": "amd64",
                    "channel": "2.1/beta",
                    "expiration-date": None,
                    "revision": 19,
                    "progressive": {"key": None, "paused": None, "percentage": None},
                    "when": "2020-02-03T20:58:37Z",
                }
            ],
            "revisions": [
                {"architectures": ["amd64"], "revision": 19, "version": "10"}
            ],
            "snap": {
                "channels": [
                    {
                        "branch": None,
                        "fallback": None,
                        "name": "2.1/stable",
                        "risk": "stable",
                        "track": "2.1",
                    },
                    {
                        "branch": None,
                        "fallback": "2.1/stable",
                        "name": "2.1/candidate",
                        "risk": "candidate",
                        "track": "2.1",
                    },
                    {
                        "branch": None,
                        "fallback": "2.1/candidate",
                        "name": "2.1/beta",
                        "risk": "beta",
                        "track": "2.1",
                    },
                    {
                        "branch": None,
                        "fallback": "2.1/beta",
                        "name": "2.1/edge",
                        "risk": "edge",
                        "track": "2.1",
                    },
                ],
                "default-track": "2.1",
            },
        }

    def test_status_without_snap_raises_exception(self):
        result = self.run_command(["status"])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(result.output, Contains("Usage:"))

    def test_status_without_login_must_ask(self):
        self.fake_store_get_snap_channel_map.mock.side_effect = [
            storeapi.errors.InvalidCredentialsError("error"),
            SnapChannelMap(self.channel_map_payload),
        ]

        result = self.run_command(
            ["status", "snap-test"], input="user@example.com\nsecret\n"
        )
        self.assertThat(
            result.output, Contains("You are required to login before continuing.")
        )

    def test_status(self):
        self.fake_store_get_snap_channel_map.mock.return_value = SnapChannelMap(
            self.channel_map_payload
        )

        result = self.run_command(["status", "snap-test"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                dedent(
                    """\
            Track    Arch    Channel    Version    Revision
            2.1      amd64   stable     -          -
                             candidate  -          -
                             beta       10         19
                             edge       ↑          ↑
            """
                )
            ),
        )

    def test_progressive_status(self):
        self.channel_map_payload["channel-map"][0]["progressive"]["percentage"] = 10.0
        self.fake_store_get_snap_channel_map.mock.return_value = SnapChannelMap(
            self.channel_map_payload
        )

        result = self.run_command(
            ["status", "snap-test", "--experimental-progressive-release"]
        )

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                dedent(
                    """\
            *EXPERIMENTAL* progressive releases in use.
            Track    Arch    Channel    Version    Revision    Progress
            2.1      amd64   stable     -          -           -
                             candidate  -          -           -
                             beta       -          -           -
                                        10         19          → 10%
                             edge       ↑          ↑           -
            """
                )
            ),
        )

    def test_status_by_arch(self):
        self.channel_map_payload["channel-map"].append(
            {
                "architecture": "s390x",
                "channel": "2.1/beta",
                "expiration-date": None,
                "revision": 99,
                "progressive": {"key": None, "paused": None, "percentage": None},
                "when": "2020-02-03T20:58:37Z",
            }
        )
        self.channel_map_payload["revisions"].append(
            {"architectures": ["s390x"], "revision": 99, "version": "10"}
        )
        self.fake_store_get_snap_channel_map.mock.return_value = SnapChannelMap(
            self.channel_map_payload
        )

        result = self.run_command(["status", "snap-test", "--arch=s390x"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                dedent(
                    """\
            Track    Arch    Channel    Version    Revision
            2.1      s390x   stable     -          -
                             candidate  -          -
                             beta       10         99
                             edge       ↑          ↑
            """
                )
            ),
        )

    def test_status_including_branch(self):
        self.channel_map_payload["channel-map"].append(
            {
                "architecture": "amd64",
                "channel": "2.1/stable/hotfix1",
                "expiration-date": None,
                "revision": 20,
                "progressive": {"key": None, "paused": None, "percentage": None},
                "when": "2020-02-03T20:58:37Z",
            }
        )
        self.channel_map_payload["revisions"].append(
            {"architectures": ["amd64"], "revision": 20, "version": "10hotfix"}
        )
        self.channel_map_payload["snap"]["channels"].append(
            {
                "branch": "hotfix1",
                "fallback": "2.1/stable",
                "name": "2.1/stable/hotfix1",
                "risk": "stable",
                "track": "2.1",
            }
        )
        self.fake_store_get_snap_channel_map.mock.return_value = SnapChannelMap(
            self.channel_map_payload
        )

        result = self.run_command(["status", "snap-test"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                dedent(
                    """\
            Track    Arch    Channel         Version    Revision
            2.1      amd64   stable          -          -
                             stable/hotfix1  10hotfix   20
                             candidate       -          -
                             beta            10         19
                             edge            ↑          ↑
            """
                )
            ),
        )

    def test_progressive_status_including_branch(self):
        self.channel_map_payload["channel-map"].append(
            {
                "architecture": "amd64",
                "channel": "2.1/stable/hotfix1",
                "expiration-date": None,
                "revision": 20,
                "progressive": {"key": None, "paused": None, "percentage": 20.0},
                "when": "2020-02-03T20:58:37Z",
            }
        )
        self.channel_map_payload["revisions"].append(
            {"architectures": ["amd64"], "revision": 20, "version": "10hotfix"}
        )
        self.channel_map_payload["snap"]["channels"].append(
            {
                "branch": "hotfix1",
                "fallback": "2.1/stable",
                "name": "2.1/stable/hotfix1",
                "risk": "stable",
                "track": "2.1",
            }
        )
        self.fake_store_get_snap_channel_map.mock.return_value = SnapChannelMap(
            self.channel_map_payload
        )

        result = self.run_command(
            ["status", "snap-test", "--experimental-progressive-release"]
        )

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                dedent(
                    """\
            *EXPERIMENTAL* progressive releases in use.
            Track    Arch    Channel         Version    Revision    Progress
            2.1      amd64   stable          -          -           -
                             stable/hotfix1  10hotfix   20          → 20%
                             candidate       -          -           -
                             beta            10         19          -
                             edge            ↑          ↑           -
            """
                )
            ),
        )
