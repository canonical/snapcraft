# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2026 Canonical Ltd.
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

from snapcraft.store.channels import Channel
from snapcraft.store.errors import ChannelNotAvailableOnArchError, InvalidChannelSet
from snapcraft.store.status import (
    SnapStatus,
    SnapStatusChannelDetails,
    SnapStatusTrackDetails,
)

TRACK_PAYLOAD = {
    "amd64": [
        {"channel": "stable", "info": "none"},
        {"channel": "candidate", "info": "tracking"},
        {
            "channel": "beta",
            "info": "specific",
            "revision": 10,
            "version": "1.0",
        },
        {
            "channel": "edge",
            "info": "specific",
            "revision": 11,
            "version": "1.1",
        },
    ],
    "arm64": [
        {"channel": "stable", "info": "none"},
        {"channel": "candidate", "info": "tracking"},
        {
            "channel": "beta",
            "info": "specific",
            "revision": 20,
            "version": "1.0",
        },
        {
            "channel": "edge",
            "info": "specific",
            "revision": 21,
            "version": "1.1",
        },
    ],
}

STATUS_PAYLOAD = {
    "channel_map_tree": {
        "latest": {
            "16": TRACK_PAYLOAD,
        },
        "9.x": {
            "16": {
                "amd64": [
                    {
                        "channel": "stable",
                        "info": "specific",
                        "revision": 5,
                        "version": "9.0",
                    },
                ]
            },
        },
    }
}


class TestSnapStatusChannelDetails:
    """Tests for SnapStatusChannelDetails."""

    def test_repr(self):
        details = SnapStatusChannelDetails(
            snap_name="my-snap",
            arch="amd64",
            payload={"channel": "stable", "info": "none"},
        )

        assert repr(details) == "<SnapStatusChannelDetails: my-snap on amd64>"

    def test_properties(self):
        details = SnapStatusChannelDetails(
            snap_name="my-snap",
            arch="arm64",
            payload={
                "channel": "edge",
                "info": "specific",
                "revision": 42,
                "version": "1.2",
            },
        )

        assert details.arch == "arm64"
        assert details.channel == "edge"
        assert details.info == "specific"
        assert details.revision == 42
        assert details.version == "1.2"

    def test_iter(self):
        """Iterator returns arch, revision, and version."""
        details = SnapStatusChannelDetails(
            snap_name="my-snap",
            arch="amd64",
            payload={
                "channel": "beta",
                "info": "specific",
                "revision": 7,
                "version": "0.9",
            },
        )

        assert list(details) == ["amd64", 7, "0.9"]

    def test_iter_with_exclude_none(self):
        """Iterator only returns non-None attributes."""
        details = SnapStatusChannelDetails(
            snap_name="my-snap",
            arch="riscv64",
            payload={"channel": "stable", "info": "none"},
        )

        assert list(details) == ["riscv64", None, None]


class TestSnapStatusTrackDetails:
    """Tests for SnapStatusTrackDetails."""

    @pytest.fixture
    def track(self):
        return SnapStatusTrackDetails(
            snap_name="my-snap", track="latest", payload=TRACK_PAYLOAD
        )

    def test_repr(self, track):
        assert repr(track) == "<SnapStatusTrackDetails: my-snap on latest>"

    def test_track_property(self, track):
        assert track.track == "latest"

    def test_get_arches(self, track):
        assert set(track.get_arches()) == {"amd64", "arm64"}

    def test_get_channel(self, track):
        channel = track.get_channel(risk="beta", arch="amd64")

        assert channel.arch == "amd64"
        assert channel.channel == "beta"
        assert channel.revision == 10
        assert channel.version == "1.0"

    def test_get_channel_no_revision(self, track):
        """Get a channel with no revisions."""
        channel = track.get_channel(risk="stable", arch="amd64")

        assert channel.arch == "amd64"
        assert channel.channel == "stable"
        assert channel.revision is None
        assert channel.version is None

    def test_get_channel_with_branch(self):
        track = SnapStatusTrackDetails(
            snap_name="my-snap",
            track="latest",
            payload={
                "amd64": [
                    {
                        "channel": "edge/pr-123",
                        "info": "specific",
                        "revision": 99,
                        "version": "dev",
                    },
                ]
            },
        )

        channel = track.get_channel(risk="edge", arch="amd64", branch="pr-123")

        assert channel.channel == "edge/pr-123"
        assert channel.revision == 99

    def test_get_channel_missing_arch_error(self, track):
        with pytest.raises(ChannelNotAvailableOnArchError):
            track.get_channel(risk="stable", arch="riscv64")

    def test_get_channel_missing_risk_error(self, track):
        with pytest.raises(ChannelNotAvailableOnArchError):
            track.get_channel(risk="stable", arch="amd64", branch="nonexistent")


class TestSnapStatus:
    """Tests for SnapStatus."""

    @pytest.fixture
    def snap_status(self):
        return SnapStatus(snap_name="my-snap", payload=STATUS_PAYLOAD)

    def test_repr(self, snap_status):
        assert repr(snap_status) == "<SnapStatus: my-snap>"

    def test_get_tracks(self, snap_status):
        assert set(snap_status.get_tracks()) == {"latest", "9.x"}

    @pytest.mark.parametrize(
        "track_name, risk, arch, expected_revision",
        [
            pytest.param("latest", "beta", "amd64", 10, id="default track"),
            pytest.param("9.x", "stable", "amd64", 5, id="other track"),
        ],
    )
    def test_get_track(self, snap_status, track_name, risk, arch, expected_revision):
        track = snap_status.get_track(track_name)
        channel = track.get_channel(risk=risk, arch=arch)

        assert isinstance(track, SnapStatusTrackDetails)
        assert track.track == track_name
        assert channel.revision == expected_revision

    def test_get_channel_set_all_arches_have_revisions(self, snap_status):
        channel_set = snap_status.get_channel_set(Channel("beta"))
        revisions = {c.revision for c in channel_set}

        assert len(channel_set) == 2
        assert revisions == {10, 20}

    def test_get_channel_set_missing_revision_error(self, snap_status):
        with pytest.raises(InvalidChannelSet):
            snap_status.get_channel_set(Channel("stable"))

    def test_get_channel_set_tracking_error(self, snap_status):
        with pytest.raises(InvalidChannelSet):
            snap_status.get_channel_set(Channel("candidate"))
