# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

import pytest
from testtools.matchers import Equals, HasLength, Is, IsInstance

from snapcraft.storeapi.v2 import channel_map
from tests import unit


class ProgressiveTest(unit.TestCase):
    def test_progressive(self):
        payload = {"paused": False, "percentage": 83.3, "current-percentage": 32.1}

        p = channel_map.Progressive.unmarshal(payload)

        self.expectThat(repr(p), Equals(f"<Progressive: 32.1=>83.3>"))
        self.expectThat(p.paused, Equals(payload["paused"]))
        self.expectThat(p.percentage, Equals(payload["percentage"]))
        self.expectThat(p.current_percentage, Equals(payload["current-percentage"]))
        self.expectThat(p.marshal(), Equals(payload))

    def test_none(self):
        payload = {"paused": None, "percentage": None, "current-percentage": None}

        p = channel_map.Progressive.unmarshal(payload)

        self.expectThat(repr(p), Equals(f"<Progressive: None=>None>"))
        self.expectThat(p.paused, Equals(payload["paused"]))
        self.expectThat(p.percentage, Equals(payload["percentage"]))
        self.expectThat(p.current_percentage, Equals(payload["current-percentage"]))
        self.expectThat(p.marshal(), Equals(payload))


class MappedChannelTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.payload = {
            "architecture": "amd64",
            "channel": "latest/stable",
            "expiration-date": None,
            "progressive": {
                "paused": None,
                "percentage": None,
                "current-percentage": None,
            },
            "revision": 2,
        }

    def test_channel(self):
        mc = channel_map.MappedChannel.unmarshal(self.payload)

        self.expectThat(
            repr(mc),
            Equals(
                "<MappedChannel: 'latest/stable' for revision 2 and architecture 'amd64'>"
            ),
        )
        self.expectThat(mc.channel, Equals(self.payload["channel"]))
        self.expectThat(mc.revision, Equals(self.payload["revision"]))
        self.expectThat(mc.architecture, Equals(self.payload["architecture"]))
        self.expectThat(mc.progressive, IsInstance(channel_map.Progressive))
        self.expectThat(mc.expiration_date, Is(None))
        self.expectThat(mc.marshal(), Equals(self.payload))

    def test_channel_with_expiration(self):
        date_string = "2020-02-11T17:51:40.891996Z"
        self.payload.update({"expiration-date": date_string})

        mc = channel_map.MappedChannel.unmarshal(self.payload)

        self.expectThat(
            repr(mc),
            Equals(
                "<MappedChannel: 'latest/stable' for revision 2 and architecture 'amd64'>"
            ),
        )
        self.expectThat(mc.channel, Equals(self.payload["channel"]))
        self.expectThat(mc.revision, Equals(self.payload["revision"]))
        self.expectThat(mc.architecture, Equals(self.payload["architecture"]))
        self.expectThat(mc.progressive, IsInstance(channel_map.Progressive))
        self.expectThat(mc.expiration_date, Equals(date_string))
        self.expectThat(mc.marshal(), Equals(self.payload))


class SnapChannelTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.payload = {
            "name": "latest/candidate",
            "track": "latest",
            "risk": "candidate",
            "branch": None,
            "fallback": None,
        }

    def test_channel(self):
        sc = channel_map.SnapChannel.unmarshal(self.payload)

        self.expectThat(repr(sc), Equals("<SnapChannel: 'latest/candidate'>"))
        self.expectThat(sc.name, Equals(self.payload["name"]))
        self.expectThat(sc.track, Equals(self.payload["track"]))
        self.expectThat(sc.risk, Equals(self.payload["risk"]))
        self.expectThat(sc.branch, Is(None))
        self.expectThat(sc.fallback, Is(None))
        self.expectThat(sc.marshal(), Equals(self.payload))

    def test_channel_with_branch(self):
        self.payload.update({"branch": "test-branch"})

        sc = channel_map.SnapChannel.unmarshal(self.payload)

        self.expectThat(repr(sc), Equals("<SnapChannel: 'latest/candidate'>"))
        self.expectThat(sc.name, Equals(self.payload["name"]))
        self.expectThat(sc.track, Equals(self.payload["track"]))
        self.expectThat(sc.risk, Equals(self.payload["risk"]))
        self.expectThat(sc.branch, Equals(self.payload["branch"]))
        self.expectThat(sc.fallback, Is(None))
        self.expectThat(sc.marshal(), Equals(self.payload))

    def test_channel_with_fallback(self):
        self.payload.update({"fallback": "latest/stable"})

        sc = channel_map.SnapChannel.unmarshal(self.payload)

        self.expectThat(repr(sc), Equals("<SnapChannel: 'latest/candidate'>"))
        self.expectThat(sc.name, Equals(self.payload["name"]))
        self.expectThat(sc.track, Equals(self.payload["track"]))
        self.expectThat(sc.risk, Equals(self.payload["risk"]))
        self.expectThat(sc.branch, Is(None)),
        self.expectThat(sc.fallback, Equals(self.payload["fallback"]))
        self.expectThat(sc.marshal(), Equals(self.payload))


_TRACK_PAYLOADS = [
    {
        "name": "latest",
        "status": "active",
        "creation-date": None,
        "version-pattern": None,
    },
    {
        "name": "1.0",
        "status": "default",
        "creation-date": "2019-10-17T14:11:59Z",
        "version-pattern": "1.*",
    },
]


@pytest.mark.parametrize("payload", _TRACK_PAYLOADS)
def test_snap_track(payload):
    st = channel_map.SnapTrack.unmarshal(payload)

    assert repr(st) == f"<SnapTrack: {st.name!r}>"
    assert st.name == payload["name"]
    assert st.status == payload["status"]
    assert st.creation_date == payload["creation-date"]
    assert st.version_pattern == payload["version-pattern"]
    assert st.marshal() == payload


class RevisionTest(unit.TestCase):
    def test_revision(self):
        payload = {"revision": 2, "version": "2.0", "architectures": ["amd64", "arm64"]}

        r = channel_map.Revision.unmarshal(payload)

        self.expectThat(
            repr(r),
            Equals(
                "<Revision: 2 for version '2.0' and architectures ['amd64', 'arm64']>"
            ),
        )
        self.expectThat(r.revision, Equals(payload["revision"]))
        self.expectThat(r.version, Equals(payload["version"]))
        self.expectThat(r.architectures, Equals(payload["architectures"]))
        self.expectThat(r.marshal(), Equals(payload))


class SnapTest(unit.TestCase):
    def test_snap(self):
        payload = {
            "name": "my-snap",
            "channels": [
                {
                    "name": "latest/stable",
                    "track": "latest",
                    "risk": "candidate",
                    "branch": None,
                    "fallback": None,
                },
                {
                    "name": "latest/candidate",
                    "track": "latest",
                    "risk": "candidate",
                    "branch": None,
                    "fallback": "latest/stable",
                },
            ],
            "tracks": [
                {
                    "name": "track1",
                    "creation-date": "2019-10-17T14:11:59Z",
                    "status": "default",
                    "version-pattern": None,
                },
                {
                    "name": "track2",
                    "creation-date": None,
                    "status": "active",
                    "version-pattern": None,
                },
            ],
        }

        s = channel_map.Snap.unmarshal(payload)

        self.expectThat(repr(s), Equals("<Snap: 'my-snap'>"))
        self.expectThat(s.name, Equals(payload["name"]))

        snap_channels = s.channels
        self.expectThat(snap_channels, HasLength(2))
        self.expectThat(snap_channels[0], IsInstance(channel_map.SnapChannel))
        self.expectThat(snap_channels[1], IsInstance(channel_map.SnapChannel))

        self.expectThat(s.marshal(), Equals(payload))


class ChannelMapTest(unit.TestCase):
    def test_channel_map(self):
        payload = {
            "channel-map": [
                {
                    "architecture": "amd64",
                    "channel": "latest/stable",
                    "expiration-date": None,
                    "progressive": {
                        "paused": None,
                        "percentage": None,
                        "current-percentage": None,
                    },
                    "revision": 2,
                },
                {
                    "architecture": "amd64",
                    "channel": "latest/stable",
                    "expiration-date": None,
                    "progressive": {
                        "paused": None,
                        "percentage": 33.3,
                        "current-percentage": 12.3,
                    },
                    "revision": 3,
                },
                {
                    "architecture": "arm64",
                    "channel": "latest/stable",
                    "expiration-date": None,
                    "progressive": {
                        "paused": None,
                        "percentage": None,
                        "current-percentage": None,
                    },
                    "revision": 2,
                },
                {
                    "architecture": "i386",
                    "channel": "latest/stable",
                    "expiration-date": None,
                    "progressive": {
                        "paused": None,
                        "percentage": None,
                        "current-percentage": None,
                    },
                    "revision": 4,
                },
            ],
            "revisions": [
                {"revision": 2, "version": "2.0", "architectures": ["amd64", "arm64"]},
                {"revision": 3, "version": "2.0", "architectures": ["amd64", "arm64"]},
                {"revision": 4, "version": "2.0", "architectures": ["i386"]},
            ],
            "snap": {
                "name": "my-snap",
                "channels": [
                    {
                        "name": "latest/stable",
                        "track": "latest",
                        "risk": "candidate",
                        "branch": None,
                        "fallback": None,
                    },
                    {
                        "name": "latest/candidate",
                        "track": "latest",
                        "risk": "candidate",
                        "branch": None,
                        "fallback": "latest/stable",
                    },
                ],
                "tracks": [
                    {
                        "name": "track1",
                        "creation-date": "2019-10-17T14:11:59Z",
                        "status": "default",
                        "version-pattern": None,
                    },
                    {
                        "name": "track2",
                        "creation-date": None,
                        "status": "active",
                        "version-pattern": None,
                    },
                ],
            },
        }

        cm = channel_map.ChannelMap.unmarshal(payload)

        # Check "channel-map".
        self.expectThat(cm.channel_map, HasLength(4))
        self.expectThat(cm.channel_map[0], IsInstance(channel_map.MappedChannel))
        self.expectThat(cm.channel_map[1], IsInstance(channel_map.MappedChannel))
        self.expectThat(cm.channel_map[2], IsInstance(channel_map.MappedChannel))
        self.expectThat(cm.channel_map[3], IsInstance(channel_map.MappedChannel))

        # Check "revisions".
        self.expectThat(cm.revisions, HasLength(3))
        self.expectThat(cm.revisions[0], IsInstance(channel_map.Revision))
        self.expectThat(cm.revisions[1], IsInstance(channel_map.Revision))
        self.expectThat(cm.revisions[2], IsInstance(channel_map.Revision))

        # Check "snap".
        self.expectThat(cm.snap, IsInstance(channel_map.Snap))

        # Marshal.
        self.expectThat(cm.marshal(), Equals(payload))

        # Test the get_mapped_channel method.
        self.expectThat(
            cm.get_mapped_channel(
                channel_name="latest/stable", architecture="amd64", progressive=False
            ),
            Equals(cm.channel_map[0]),
        )
        self.expectThat(
            cm.get_mapped_channel(
                channel_name="latest/stable", architecture="amd64", progressive=True
            ),
            Equals(cm.channel_map[1]),
        )
        self.assertRaises(
            ValueError,
            cm.get_mapped_channel,
            channel_name="latest/stable",
            architecture="arm64",
            progressive=True,
        )
        self.assertRaises(
            ValueError,
            cm.get_mapped_channel,
            channel_name="latest/stable",
            architecture="i386",
            progressive=True,
        )

        # Test the get_channel_info method.
        self.expectThat(
            cm.get_channel_info("latest/stable"), Equals(cm.snap.channels[0])
        )
        self.assertRaises(ValueError, cm.get_channel_info, "other-track/stable")

        # Test the get_revision method.
        self.expectThat(cm.get_revision(4), Equals(cm.revisions[2]))
        self.assertRaises(ValueError, cm.get_revision, 5)

        # Test the get_existing_architectures method.
        self.expectThat(
            cm.get_existing_architectures(), Equals(set(["arm64", "amd64", "i386"]))
        )
