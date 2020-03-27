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


class ReleaseCommandTestCase(FakeStoreCommandsBaseTestCase):
    def setUp(self):
        super().setUp()

        self.fake_store_release.mock.return_value = {"opened_channels": ["beta"]}
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

    def test_release_without_snap_name_must_raise_exception(self):
        result = self.run_command(["release"])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(result.output, Contains("Usage:"))

    def test_release(self):
        self.fake_store_release.mock.return_value = {"opened_channels": ["2.1/beta"]}
        self.fake_store_get_snap_channel_map.mock.return_value = SnapChannelMap(
            self.channel_map_payload
        )

        result = self.run_command(["release", "nil-snap", "19", "2.1/beta"])

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
            The '2.1/beta' channel is now open.
            """
                )
            ),
        )
        self.fake_store_release.mock.assert_called_once_with(
            snap_name="nil-snap",
            revision="19",
            channels=["2.1/beta"],
            progressive_percentage=None,
        )

    def test_progressive_release(self):
        self.fake_store_release.mock.return_value = {"opened_channels": ["2.1/beta"]}
        self.channel_map_payload["channel-map"][0]["progressive"]["percentage"] = 10.0
        self.fake_store_get_snap_channel_map.mock.return_value = SnapChannelMap(
            self.channel_map_payload
        )

        result = self.run_command(
            [
                "release",
                "nil-snap",
                "19",
                "2.1/beta",
                "--progressive",
                "10",
                "--experimental-progressive-release",
            ]
        )

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
            The '2.1/beta' channel is now open.
            """
                )
            ),
        )
        self.fake_store_release.mock.assert_called_once_with(
            snap_name="nil-snap",
            revision="19",
            channels=["2.1/beta"],
            progressive_percentage=10,
        )

    def test_release_with_branch(self):
        self.fake_store_release.mock.return_value = {
            "opened_channels": ["stable/hotfix1"]
        }
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

        result = self.run_command(["release", "nil-snap", "20", "2.1/stable/hotfix1"])

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
            The 'stable/hotfix1' channel is now open.
                    """
                )
            ),
        )

        self.fake_store_release.mock.assert_called_once_with(
            snap_name="nil-snap",
            revision="20",
            channels=["2.1/stable/hotfix1"],
            progressive_percentage=None,
        )

    def test_progressive_release_with_branch(self):
        self.fake_store_release.mock.return_value = {
            "opened_channels": ["2.1/stable/hotfix1"]
        }
        self.channel_map_payload["channel-map"].append(
            {
                "architecture": "amd64",
                "channel": "2.1/stable/hotfix1",
                "expiration-date": None,
                "revision": 20,
                "progressive": {"key": None, "paused": None, "percentage": 80.0},
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
            [
                "release",
                "--progressive",
                "80",
                "--experimental-progressive-release",
                "nil-snap",
                "20",
                "2.1/stable/hotfix1",
            ]
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
                             stable/hotfix1  10hotfix   20          → 80%
                             candidate       -          -           -
                             beta            10         19          -
                             edge            ↑          ↑           -
            The '2.1/stable/hotfix1' channel is now open.
                    """
                )
            ),
        )
        self.fake_store_release.mock.assert_called_once_with(
            snap_name="nil-snap",
            revision="20",
            channels=["2.1/stable/hotfix1"],
            progressive_percentage=80,
        )

    def test_release_without_login_must_ask(self):
        self.fake_store_release.mock.side_effect = [
            storeapi.errors.InvalidCredentialsError("error"),
            {"opened_channels": ["beta"]},
        ]

        result = self.run_command(
            ["release", "nil-snap", "19", "beta"], input="user@example.com\nsecret\n"
        )
        self.assertThat(
            result.output, Contains("You are required to login before continuing.")
        )
