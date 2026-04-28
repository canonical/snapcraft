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

from snapcraft import errors
from snapcraft.store.channels import Channel


@pytest.mark.parametrize(
    ("channel", "expected_track", "expected_risk", "expected_branch"),
    [
        # default (latest) track
        ("stable", "latest", "stable", None),
        ("candidate", "latest", "candidate", None),
        ("beta", "latest", "beta", None),
        ("edge", "latest", "edge", None),
        # custom tracks
        ("9.x/stable", "9.x", "stable", None),
        ("9.x/candidate", "9.x", "candidate", None),
        ("9.x/beta", "9.x", "beta", None),
        ("9.x/edge", "9.x", "edge", None),
        # branches
        ("edge/pr-123", "latest", "edge", "pr-123"),
        ("9.x/edge/pr-123", "9.x", "edge", "pr-123"),
    ],
)
def test_valid_channel(channel, expected_track, expected_risk, expected_branch):
    actual_channel = Channel(channel)

    assert actual_channel.track == expected_track
    assert actual_channel.risk == expected_risk
    assert actual_channel.branch == expected_branch


@pytest.mark.parametrize(
    "channel",
    [
        pytest.param("invalid", id="invalid risk"),
        pytest.param("track/invalid", id="invalid risk with track"),
        pytest.param("track/invalid/branch", id="invalid risk with track and branch"),
        pytest.param("a/b/c/d", id="too many parts"),
        pytest.param("", id="empty"),
    ],
)
def test_invalid_channel(channel):
    with pytest.raises(errors.SnapcraftError, match="Channel logic failed for"):
        Channel(channel)


@pytest.mark.parametrize("channel", ["stable", "9.x/edge/pr-123"])
def test_str(channel):
    assert str(Channel(channel)) == channel


@pytest.mark.parametrize("channel", ["stable", "9.x/edge/pr-123"])
def test_repr(channel):
    assert repr(Channel(channel)) == repr(channel)


@pytest.mark.parametrize(
    ("channel_a", "channel_b"),
    [
        pytest.param("stable", "stable", id="equal with self simple"),
        pytest.param("9.x/beta", "9.x/beta", id="equal with track"),
        pytest.param("9.x/edge/pr-123", "9.x/edge/pr-123", id="equal with branch"),
        pytest.param("latest/stable", "stable", id="implicit latest"),
        pytest.param(
            "latest/edge/pr-123", "edge/pr-123", id="implicit latest with branch"
        ),
    ],
)
def test_equal_channels(channel_a, channel_b):
    assert Channel(channel_a) == Channel(channel_b)


@pytest.mark.parametrize(
    ("channel_a", "channel_b"),
    [
        pytest.param("9.x/stable", "latest/stable", id="different track"),
        pytest.param("9.x/stable", "stable", id="implicit different track"),
        pytest.param("stable", "edge", id="different risk"),
        pytest.param("9.x/edge/pr-123", "9.x/edge/pr-456", id="different branch"),
    ],
)
def test_unequal_channels(channel_a, channel_b):
    assert Channel(channel_a) != Channel(channel_b)


def test_not_equal_to_non_channel():
    result = Channel("stable").__eq__("stable")
    assert result is NotImplemented


@pytest.mark.parametrize(
    ("track", "risk", "branch", "expected_str"),
    [
        pytest.param("", "stable", None, "stable", id="default track"),
        pytest.param(
            "", "edge", "pr-123", "edge/pr-123", id="default track and branch"
        ),
        pytest.param("9.x", "candidate", None, "9.x/candidate", id="track"),
        pytest.param("9.x", "edge", "pr-123", "9.x/edge/pr-123", id="track and branch"),
        pytest.param(
            "latest",
            "edge",
            "pr-123",
            "latest/edge/pr-123",
            id="latest track and branch",
        ),
    ],
)
def test_from_tuple(track, risk, branch, expected_str):
    channel = Channel.from_channel_tuple(track=track, risk=risk, branch=branch)

    assert channel.track == branch or "latest"
    assert channel.risk == risk
    assert channel.branch == branch
    assert str(channel) == expected_str


def test_from_tuple_no_risk_error():
    with pytest.raises(errors.SnapcraftError, match="Incorrect channel tuple"):
        Channel.from_channel_tuple(track="", risk="", branch=None)
