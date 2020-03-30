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

from testtools.matchers import Equals

import snapcraft.storeapi.errors
from snapcraft.storeapi.v2.snap_channel_map import SnapChannelMap
from . import FakeStoreCommandsBaseTestCase


class CloseCommandTestCase(FakeStoreCommandsBaseTestCase):
    def setUp(self):
        super().setUp()

        self.channel_map_payload = {
            "channel-map": [
                {
                    "architecture": "amd64",
                    "channel": "latest/edge",
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
                        "name": "latest/stable",
                        "risk": "stable",
                        "track": "latest",
                    },
                    {
                        "branch": None,
                        "fallback": "latest/stable",
                        "name": "latest/candidate",
                        "risk": "candidate",
                        "track": "latest",
                    },
                    {
                        "branch": None,
                        "fallback": "latest/candidate",
                        "name": "latest/beta",
                        "risk": "beta",
                        "track": "latest",
                    },
                    {
                        "branch": None,
                        "fallback": "latest/beta",
                        "name": "latest/edge",
                        "risk": "edge",
                        "track": "latest",
                    },
                ]
            },
        }
        self.fake_store_get_snap_channel_map.mock.return_value = SnapChannelMap(
            self.channel_map_payload
        )

    def test_close_missing_permission(self):
        raised = self.assertRaises(
            snapcraft.storeapi.errors.StoreChannelClosingPermissionError,
            self.run_command,
            ["close", "foo", "beta"],
        )

        self.assertThat(
            str(raised),
            Equals(
                "Your account lacks permission to close channels for this snap. "
                "Make sure the logged in account has upload permissions on "
                "'foo' in series '16'."
            ),
        )

    def test_close_basic(self):
        result = self.run_command(["close", "basic", "beta"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                dedent(
                    """\
            Track    Arch    Channel    Version    Revision
            latest   amd64   stable     -          -
                             candidate  -          -
                             beta       -          -
                             edge       10         19
            The beta channel is now closed.
            """
                )
            ),
        )

    def test_close_multiple_channels(self):
        self.channel_map_payload["channel-map"].append(
            {
                "architecture": "s390x",
                "channel": "2.1/edge",
                "expiration-date": None,
                "revision": 20,
                "progressive": {"key": None, "paused": None, "percentage": None},
                "when": "2020-02-03T20:58:37Z",
            }
        )
        self.channel_map_payload["revisions"].append(
            {"architectures": ["s390x"], "revision": 20, "version": "10"}
        )
        self.channel_map_payload["snap"]["channels"].extend(
            [
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
            ]
        )
        self.fake_store_get_snap_channel_map.mock.return_value = SnapChannelMap(
            self.channel_map_payload
        )
        self.fake_store_close.mock.side_effect = [(["beta", "2.1/beta"], dict())]
        result = self.run_command(["close", "basic", "latest/beta", "2.1/beta"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(
            result.output,
            Equals(
                dedent(
                    """\
           Track    Arch    Channel    Version    Revision
           latest   amd64   stable     -          -
                            candidate  -          -
                            beta       -          -
                            edge       10         19
           2.1      s390x   stable     -          -
                            candidate  -          -
                            beta       -          -
                            edge       10         20
           The beta and 2.1/beta channels are now closed.
           """
                )
            ),
        )
