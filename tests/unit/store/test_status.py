# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

from testtools.matchers import Equals, HasLength

from snapcraft.storeapi import channels, errors, status
from tests import unit


class TestSnapStatusChannelDetails:
    scenarios = (
        (
            "specific",
            dict(
                payload={
                    "version": "3.4",
                    "channel": "stable",
                    "info": "specific",
                    "revision": 1,
                },
                expected_version="3.4",
                expected_channel="stable",
                expected_info="specific",
                expected_revision=1,
            ),
        ),
        (
            "tracking",
            dict(
                payload={"channel": "candidate", "info": "tracking"},
                expected_version=None,
                expected_channel="candidate",
                expected_info="tracking",
                expected_revision=None,
            ),
        ),
        (
            "specific",
            dict(
                payload={"channel": "stable", "info": "none"},
                expected_version=None,
                expected_channel="stable",
                expected_info="none",
                expected_revision=None,
            ),
        ),
    )

    def test_properties(
        self,
        payload,
        expected_version,
        expected_channel,
        expected_info,
        expected_revision,
    ):
        s = status.SnapStatusChannelDetails(
            snap_name="test-snap", arch="arm64", payload=payload
        )

        assert s.arch == "arm64"
        assert s.version == expected_version
        assert s.channel == expected_channel
        assert s.info == expected_info
        assert repr(s) == "<SnapStatusChannelDetails: test-snap on arm64>"

        assert list(s) == ["arm64", expected_revision, expected_version]


class SnapStatusTrackTest(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.status = status.SnapStatusTrackDetails(
            snap_name="test-snap",
            track="latest",
            payload={
                "ppc64el": [
                    {
                        "version": "3.4",
                        "channel": "stable",
                        "info": "specific",
                        "revision": 2833,
                    },
                    {"channel": "candidate", "info": "tracking"},
                    {"channel": "beta", "info": "tracking"},
                    {
                        "version": "3.4+git11.g8d7f165",
                        "channel": "edge",
                        "info": "specific",
                        "revision": 2895,
                    },
                ]
            },
        )

    def test_properties(self):
        self.assertThat(self.status.track, Equals("latest"))

    def test_repr(self):
        self.assertThat(
            repr(self.status), Equals("<SnapStatusTrackDetails: test-snap on latest>")
        )

    def test_get_arches(self):
        self.assertThat(self.status.get_arches(), Equals(["ppc64el"]))

    def test_get_channel(self):
        c = self.status.get_channel(risk="candidate", arch="ppc64el")
        self.assertThat(c.arch, Equals("ppc64el"))
        self.assertThat(c.version, Equals(None))
        self.assertThat(c.channel, Equals("candidate"))
        self.assertThat(c.revision, Equals(None))

    def test_get_missing_arch(self):
        self.assertRaises(
            errors.ChannelNotAvailableOnArchError,
            self.status.get_channel,
            risk="candidate",
            arch="amd64",
        )

    def test_get_missing_branch(self):
        self.assertRaises(
            errors.ChannelNotAvailableOnArchError,
            self.status.get_channel,
            risk="candidate",
            branch="patch-1",
            arch="ppcel64",
        )


class SnapStatusTest(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.status = status.SnapStatus(
            snap_name="test-snap",
            payload={
                "channel_map_tree": {
                    "latest": {
                        "16": {
                            "amd64": [
                                {"channel": "stable", "info": "none"},
                                {"channel": "candidate", "info": "none"},
                                {"channel": "beta", "info": "none"},
                                {
                                    "channel": "edge",
                                    "info": "specific",
                                    "revision": 1,
                                    "version": "0.1",
                                },
                            ]
                        }
                    }
                }
            },
        )

    def test_repr(self):
        self.assertThat(repr(self.status), Equals("<SnapStatus: test-snap>"))

    def test_get_tracks(self):
        self.assertThat(self.status.get_tracks(), Equals(["latest"]))

    def test_get_track(self):
        track_details = self.status.get_track("latest")
        self.assertThat(track_details.track, Equals("latest"))
        self.assertThat(
            track_details.get_channel(risk="stable", arch="amd64").channel,
            Equals("stable"),
        )

    def test_get_channel_set(self):
        channel_set = self.status.get_channel_set(channels.Channel("edge"))
        self.assertThat(channel_set, HasLength(1))

    def test_get_tracking_channel_set(self):
        self.assertRaises(
            errors.InvalidChannelSet,
            self.status.get_channel_set,
            channels.Channel("candidate"),
        )

    def test_get_none_channel_set(self):
        self.assertRaises(
            errors.InvalidChannelSet,
            self.status.get_channel_set,
            channels.Channel("stable"),
        )
