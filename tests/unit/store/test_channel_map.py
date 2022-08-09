# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020, 2022 Canonical Ltd
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

from snapcraft.store import channel_map

############
# Fixtures #
############


@pytest.fixture
def channel_payload():
    return {
        "name": "latest/candidate",
        "track": "latest",
        "risk": "candidate",
        "branch": None,
        "fallback": None,
    }


@pytest.fixture
def mapped_channel_payload():
    return {
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


#########
# Tests #
#########


def test_progressive():
    payload = {"paused": False, "percentage": 83.3, "current-percentage": 32.1}

    p = channel_map.Progressive.unmarshal(payload)

    assert repr(p) == "<Progressive: 32.1=>83.3>"
    assert p.paused == payload["paused"]
    assert p.percentage == payload["percentage"]
    assert p.current_percentage == payload["current-percentage"]
    assert p.marshal() == payload


def test_none():
    payload = {"paused": None, "percentage": None, "current-percentage": None}

    p = channel_map.Progressive.unmarshal(payload)

    assert repr(p) == "<Progressive: None=>None>"
    assert p.paused == payload["paused"]
    assert p.percentage == payload["percentage"]
    assert p.current_percentage == payload["current-percentage"]
    assert p.marshal() == payload


def test_mapped_channel(mapped_channel_payload):
    mc = channel_map.MappedChannel.unmarshal(mapped_channel_payload)

    assert (
        repr(mc)
    ) == "<MappedChannel: 'latest/stable' for revision 2 and architecture 'amd64'>"
    assert mc.channel == mapped_channel_payload["channel"]
    assert mc.revision == mapped_channel_payload["revision"]
    assert mc.architecture == mapped_channel_payload["architecture"]
    assert isinstance(mc.progressive, channel_map.Progressive)
    assert mc.expiration_date is None
    assert mc.marshal() == mapped_channel_payload


def test_snap_channel_with_expiration(mapped_channel_payload):
    date_string = "2020-02-11T17:51:40.891996Z"
    mapped_channel_payload.update({"expiration-date": date_string})

    mc = channel_map.MappedChannel.unmarshal(mapped_channel_payload)

    assert (
        repr(mc)
    ) == "<MappedChannel: 'latest/stable' for revision 2 and architecture 'amd64'>"
    assert mc.channel == mapped_channel_payload["channel"]
    assert mc.revision == mapped_channel_payload["revision"]
    assert mc.architecture == mapped_channel_payload["architecture"]
    assert isinstance(mc.progressive, channel_map.Progressive)
    assert mc.expiration_date == date_string
    assert mc.marshal() == mapped_channel_payload


def test_snap_channel(channel_payload):
    sc = channel_map.SnapChannel.unmarshal(channel_payload)

    assert repr(sc) == "<SnapChannel: 'latest/candidate'>"
    assert sc.name == channel_payload["name"]
    assert sc.track == channel_payload["track"]
    assert sc.risk == channel_payload["risk"]
    assert sc.branch is None
    assert sc.fallback is None
    assert sc.marshal() == channel_payload


def test_snap_channel_with_branch(channel_payload):
    channel_payload.update({"branch": "test-branch"})

    sc = channel_map.SnapChannel.unmarshal(channel_payload)

    assert repr(sc) == "<SnapChannel: 'latest/candidate'>"
    assert sc.name == channel_payload["name"]
    assert sc.track == channel_payload["track"]
    assert sc.risk == channel_payload["risk"]
    assert sc.branch == channel_payload["branch"]
    assert sc.fallback is None
    assert sc.marshal() == channel_payload


def test_snap_channel_with_fallback(channel_payload):
    channel_payload.update({"fallback": "latest/stable"})

    sc = channel_map.SnapChannel.unmarshal(channel_payload)

    assert repr(sc) == "<SnapChannel: 'latest/candidate'>"
    assert sc.name == channel_payload["name"]
    assert sc.track == channel_payload["track"]
    assert sc.risk == channel_payload["risk"]
    assert sc.branch is None
    assert sc.fallback == channel_payload["fallback"]
    assert sc.marshal() == channel_payload


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


def test_revision():
    payload = {"revision": 2, "version": "2.0", "architectures": ["amd64", "arm64"]}

    r = channel_map.Revision.unmarshal(payload)

    assert repr(r) == (
        "<Revision: 2 for version '2.0' and architectures ['amd64', 'arm64']>"
    )
    assert r.revision == payload["revision"]
    assert r.version == payload["version"]
    assert r.architectures == payload["architectures"]
    assert r.marshal() == payload


def test_snap():
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

    assert repr(s) == "<Snap: 'my-snap'>"
    assert s.name == payload["name"]

    snap_channels = s.channels
    assert len(snap_channels) == 2
    assert isinstance(snap_channels[0], channel_map.SnapChannel)
    assert isinstance(snap_channels[1], channel_map.SnapChannel)

    assert s.marshal() == payload


def test_channel_map():
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
    assert len(cm.channel_map) == 4
    assert isinstance(cm.channel_map[0], channel_map.MappedChannel)
    assert isinstance(cm.channel_map[1], channel_map.MappedChannel)
    assert isinstance(cm.channel_map[2], channel_map.MappedChannel)
    assert isinstance(cm.channel_map[3], channel_map.MappedChannel)

    # Check "revisions".
    assert len(cm.revisions) == 3
    assert isinstance(cm.revisions[0], channel_map.Revision)
    assert isinstance(cm.revisions[1], channel_map.Revision)
    assert isinstance(cm.revisions[2], channel_map.Revision)

    # Check "snap".
    assert isinstance(cm.snap, channel_map.Snap)

    # Marshal.
    assert cm.marshal() == payload

    # Test the get_mapped_channel method.
    assert (
        cm.get_mapped_channel(
            channel_name="latest/stable", architecture="amd64", progressive=False
        )
    ) == cm.channel_map[0]
    assert (
        cm.get_mapped_channel(
            channel_name="latest/stable", architecture="amd64", progressive=True
        )
    ) == cm.channel_map[1]
    with pytest.raises(ValueError):
        cm.get_mapped_channel(
            channel_name="latest/stable",
            architecture="arm64",
            progressive=True,
        )
    with pytest.raises(ValueError):
        cm.get_mapped_channel(
            channel_name="latest/stable",
            architecture="i386",
            progressive=True,
        )

    # Test the get_channel_info method.
    assert cm.get_channel_info("latest/stable") == cm.snap.channels[0]
    with pytest.raises(ValueError):
        cm.get_channel_info("other-track/stable")

    # Test the get_revision method.
    assert cm.get_revision(4) == cm.revisions[2]
    with pytest.raises(ValueError):
        cm.get_revision(5)

    # Test the get_existing_architectures method.
    assert cm.get_existing_architectures() == set(["arm64", "amd64", "i386"])
