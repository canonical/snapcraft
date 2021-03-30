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
from snapcraft.storeapi.v2.channel_map import (
    MappedChannel,
    Progressive,
    Revision,
    SnapChannel,
)

from . import FakeStoreCommandsBaseTestCase


class ReleaseCommandTestCase(FakeStoreCommandsBaseTestCase):
    def setUp(self):
        super().setUp()

        self.fake_store_release.mock.return_value = {"opened_channels": ["2.1/beta"]}

    def test_release_without_snap_name_must_raise_exception(self):
        result = self.run_command(["release"])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(result.output, Contains("Usage:"))

    def test_release(self):
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
        self.channel_map.channel_map[0].progressive.percentage = 10.0
        self.channel_map.channel_map[0].progressive.current_percentage = 5.0

        result = self.run_command(
            [
                "release",
                "nil-snap",
                "19",
                "2.1/beta",
                "--progressive",
                "10",
                "--experimental-progressive-releases",
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
                                        10         19          5→10%
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
        self.channel_map.channel_map.append(
            MappedChannel(
                channel="2.1/stable/hotfix1",
                architecture="amd64",
                expiration_date="2020-02-03T20:58:37Z",
                revision=20,
                progressive=Progressive(
                    paused=None, percentage=None, current_percentage=None
                ),
            )
        )
        self.channel_map.revisions.append(
            Revision(architectures=["amd64"], revision=20, version="10hotfix")
        )
        self.channel_map.snap.channels.append(
            SnapChannel(
                name="2.1/stable/hotfix1",
                track="2.1",
                risk="stable",
                branch="hotfix1",
                fallback="2.1/stable",
            )
        )

        result = self.run_command(["release", "nil-snap", "20", "2.1/stable/hotfix1"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                dedent(
                    """\
            Track    Arch    Channel         Version    Revision    Expires at
            2.1      amd64   stable          -          -
                             stable/hotfix1  10hotfix   20          2020-02-03T20:58:37Z
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
        self.channel_map.channel_map.append(
            MappedChannel(
                channel="2.1/stable/hotfix1",
                architecture="amd64",
                expiration_date="2020-02-03T20:58:37Z",
                revision=20,
                progressive=Progressive(
                    paused=None, percentage=80.0, current_percentage=None
                ),
            )
        )
        self.channel_map.revisions.append(
            Revision(architectures=["amd64"], revision=20, version="10hotfix")
        )
        self.channel_map.snap.channels.append(
            SnapChannel(
                name="2.1/stable/hotfix1",
                track="2.1",
                risk="stable",
                branch="hotfix1",
                fallback="2.1/stable",
            )
        )

        result = self.run_command(
            [
                "release",
                "--progressive",
                "80",
                "--experimental-progressive-releases",
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
            Track    Arch    Channel         Version    Revision    Progress    Expires at
            2.1      amd64   stable          -          -           -
                             stable/hotfix1  10hotfix   20          ?→80%       2020-02-03T20:58:37Z
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

    def test_progressive_release_with_null_current_percentage(self):
        self.channel_map.channel_map[0].progressive.percentage = 10.0
        self.channel_map.channel_map[0].progressive.current_percentage = None

        result = self.run_command(
            [
                "release",
                "nil-snap",
                "19",
                "2.1/beta",
                "--progressive",
                "10",
                "--experimental-progressive-releases",
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
                                        10         19          ?→10%
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

    def test_release_without_login_must_ask(self):
        self.fake_store_release.mock.side_effect = [
            storeapi.http_clients.errors.InvalidCredentialsError("error"),
            {"opened_channels": ["beta"]},
        ]

        result = self.run_command(
            ["release", "nil-snap", "19", "beta"], input="user@example.com\nsecret\n"
        )
        self.assertThat(
            result.output, Contains("You are required to login before continuing.")
        )
