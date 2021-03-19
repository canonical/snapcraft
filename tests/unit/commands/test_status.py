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


class StatusCommandTestCase(FakeStoreCommandsBaseTestCase):
    def test_status_without_snap_raises_exception(self):
        result = self.run_command(["status"])

        self.assertThat(result.exit_code, Equals(2))
        self.assertThat(result.output, Contains("Usage:"))

    def test_status_without_login_must_ask(self):
        self.fake_store_get_snap_channel_map.mock.side_effect = [
            storeapi.http_clients.errors.InvalidCredentialsError("error"),
            self.channel_map,
        ]

        result = self.run_command(
            ["status", "snap-test"], input="user@example.com\nsecret\n"
        )
        self.assertThat(
            result.output, Contains("You are required to login before continuing.")
        )

    def test_status(self):
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
            2.0      amd64   stable     -          -
                             candidate  -          -
                             beta       10         18
                             edge       ↑          ↑
            """
                )
            ),
        )

    def test_status_following(self):
        self.channel_map.channel_map = [
            MappedChannel(
                channel="2.1/stable",
                architecture="amd64",
                expiration_date="2020-02-03T20:58:37Z",
                revision=20,
                progressive=Progressive(
                    paused=None, percentage=None, current_percentage=None
                ),
            )
        ]
        self.channel_map.revisions.append(
            Revision(architectures=["amd64"], revision=20, version="10")
        )

        result = self.run_command(["status", "snap-test"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                dedent(
                    """\
            Track    Arch    Channel    Version    Revision
            2.1      amd64   stable     10         20
                             candidate  ↑          ↑
                             beta       ↑          ↑
                             edge       ↑          ↑
            """
                )
            ),
        )

    def test_status_no_releases(self):
        self.channel_map.channel_map = []

        result = self.run_command(["status", "snap-test"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output.strip(), Equals("This snap has no released revisions.")
        )

    def test_progressive_status(self):
        self.channel_map.channel_map.append(
            MappedChannel(
                channel="2.1/beta",
                architecture="amd64",
                expiration_date="2020-02-03T20:58:37Z",
                revision=20,
                progressive=Progressive(
                    paused=None, percentage=10.0, current_percentage=7.2
                ),
            )
        )
        self.channel_map.revisions.append(
            Revision(architectures=["amd64"], revision=20, version="11")
        )

        result = self.run_command(
            ["status", "snap-test", "--experimental-progressive-releases"]
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
                             beta       10         19          93→90%
                                        11         20          7→10%
                             edge       ↑          ↑           -
            2.0      amd64   stable     -          -           -
                             candidate  -          -           -
                             beta       10         18          -
                             edge       ↑          ↑           -
            """
                )
            ),
        )

    def test_status_by_arch(self):
        self.channel_map.channel_map.append(
            MappedChannel(
                channel="2.1/beta",
                architecture="s390x",
                expiration_date=None,
                revision=99,
                progressive=Progressive(
                    paused=None, percentage=None, current_percentage=None
                ),
            )
        )
        self.channel_map.revisions.append(
            Revision(architectures=["s390x"], revision=99, version="10")
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

    def test_status_by_multiple_arch(self):
        self.channel_map.channel_map.append(
            MappedChannel(
                channel="2.1/beta",
                architecture="s390x",
                expiration_date=None,
                revision=98,
                progressive=Progressive(
                    paused=None, percentage=None, current_percentage=None
                ),
            )
        )
        self.channel_map.channel_map.append(
            MappedChannel(
                channel="2.1/beta",
                architecture="arm64",
                expiration_date=None,
                revision=99,
                progressive=Progressive(
                    paused=None, percentage=None, current_percentage=None
                ),
            )
        )
        self.channel_map.revisions.append(
            Revision(architectures=["s390x"], revision=98, version="10")
        )
        self.channel_map.revisions.append(
            Revision(architectures=["arm64"], revision=99, version="10")
        )

        result = self.run_command(
            ["status", "snap-test", "--arch=s390x", "--arch=arm64"]
        )

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                dedent(
                    """\
            Track    Arch    Channel    Version    Revision
            2.1      arm64   stable     -          -
                             candidate  -          -
                             beta       10         99
                             edge       ↑          ↑
                     s390x   stable     -          -
                             candidate  -          -
                             beta       10         98
                             edge       ↑          ↑
            """
                )
            ),
        )

    def test_status_by_track(self):
        result = self.run_command(["status", "snap-test", "--track=2.0"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                dedent(
                    """\
            Track    Arch    Channel    Version    Revision
            2.0      amd64   stable     -          -
                             candidate  -          -
                             beta       10         18
                             edge       ↑          ↑
            """
                )
            ),
        )

    def test_status_by_multiple_track(self):
        result = self.run_command(["status", "snap-test", "--track=2.0", "--track=2.1"])

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
            2.0      amd64   stable     -          -
                             candidate  -          -
                             beta       10         18
                             edge       ↑          ↑
            """
                )
            ),
        )

    def test_status_by_track_and_arch(self):
        self.channel_map.channel_map.append(
            MappedChannel(
                channel="2.1/beta",
                architecture="s390x",
                expiration_date=None,
                revision=99,
                progressive=Progressive(
                    paused=None, percentage=None, current_percentage=None
                ),
            )
        )
        self.channel_map.channel_map.append(
            MappedChannel(
                channel="2.0/beta",
                architecture="s390x",
                expiration_date=None,
                revision=98,
                progressive=Progressive(
                    paused=None, percentage=None, current_percentage=None
                ),
            )
        )
        self.channel_map.revisions.append(
            Revision(architectures=["s390x"], revision=99, version="10")
        )
        self.channel_map.revisions.append(
            Revision(architectures=["s390x"], revision=98, version="10")
        )

        result = self.run_command(
            ["status", "snap-test", "--arch=s390x", "--track=2.0"]
        )

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                dedent(
                    """\
            Track    Arch    Channel    Version    Revision
            2.0      s390x   stable     -          -
                             candidate  -          -
                             beta       10         98
                             edge       ↑          ↑
            """
                )
            ),
        )

    def test_status_including_branch(self):
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

        result = self.run_command(["status", "snap-test"])

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
            2.0      amd64   stable          -          -
                             candidate       -          -
                             beta            10         18
                             edge            ↑          ↑
            """
                )
            ),
        )

    def test_progressive_status_including_branch(self):
        self.channel_map.channel_map.append(
            MappedChannel(
                channel="2.1/stable/hotfix1",
                architecture="amd64",
                expiration_date="2020-02-03T20:58:37Z",
                revision=20,
                progressive=Progressive(
                    paused=None, percentage=20.0, current_percentage=12.3
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
            ["status", "snap-test", "--experimental-progressive-releases"]
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
                             stable/hotfix1  10hotfix   20          12→20%      2020-02-03T20:58:37Z
                             candidate       -          -           -
                             beta            10         19          -
                             edge            ↑          ↑           -
            2.0      amd64   stable          -          -           -
                             candidate       -          -           -
                             beta            10         18          -
                             edge            ↑          ↑           -
            """
                )
            ),
        )

    def test_progressive_status_with_null_current_percentage(self):
        self.channel_map.channel_map[0].progressive.percentage = 10.0
        self.channel_map.channel_map[0].progressive.current_percentage = None

        result = self.run_command(
            ["status", "snap-test", "--experimental-progressive-releases"]
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
                                        10         19          ?→10%
                             edge       ↑          ↑           -
            2.0      amd64   stable     -          -           -
                             candidate  -          -           -
                             beta       10         18          -
                             edge       ↑          ↑           -
            """
                )
            ),
        )
