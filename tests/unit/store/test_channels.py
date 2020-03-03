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

import pytest

from snapcraft.storeapi import channels


class TestChannel:
    scenarios = (
        (
            "risk",
            dict(
                channel="edge",
                expected_track="latest",
                expected_risk="edge",
                expected_branch=None,
                expected_channel="edge",
            ),
        ),
        (
            "track/risk",
            dict(
                channel="11/candidate",
                expected_track="11",
                expected_risk="candidate",
                expected_branch=None,
                expected_channel="11/candidate",
            ),
        ),
        (
            "track/risk/branch",
            dict(
                channel="11/edge/patch-1",
                expected_track="11",
                expected_risk="edge",
                expected_branch="patch-1",
                expected_channel="11/edge/patch-1",
            ),
        ),
        (
            "risk/branch",
            dict(
                channel="edge/patch-2",
                expected_track="latest",
                expected_risk="edge",
                expected_branch="patch-2",
                expected_channel="edge/patch-2",
            ),
        ),
    )

    def test_channel(
        self, channel, expected_track, expected_risk, expected_branch, expected_channel
    ):
        c = channels.Channel(channel)

        assert c.track == expected_track
        assert c.risk == expected_risk
        assert c.branch == expected_branch
        assert str(c) == expected_channel
        assert repr(c), "'{}'".format(expected_channel)


class TestChannelTuple:
    scenarios = (
        (
            "risk",
            dict(
                track=None,
                risk="edge",
                branch=None,
                expected_track="latest",
                expected_channel="edge",
            ),
        ),
        (
            "track/risk",
            dict(
                track="11",
                risk="candidate",
                branch=None,
                expected_track="11",
                expected_channel="11/candidate",
            ),
        ),
        (
            "track/risk/branch",
            dict(
                track="11",
                risk="edge",
                branch="patch-1",
                expected_track="11",
                expected_channel="11/edge/patch-1",
            ),
        ),
        (
            "risk/branch",
            dict(
                track="latest",
                risk="edge",
                branch="patch-2",
                expected_track="latest",
                expected_channel="latest/edge/patch-2",
            ),
        ),
    )

    def test_channel(self, track, risk, branch, expected_track, expected_channel):
        c = channels.Channel.from_channel_tuple(track=track, risk=risk, branch=branch)
        assert c.track == expected_track
        assert c.risk == risk
        assert c.branch == branch
        assert str(c) == expected_channel
        assert repr(c), "'{}'".format(expected_channel)


class TestInvalidChannel:
    scenarios = (
        ("invalid risk", dict(channel="invalid-edge")),
        ("invalid track/risk", dict(channel="track/invalid-edge")),
        ("invalid track/risk/branch", dict(channel="track/invalid-edge/branch")),
        ("invalid tuple", dict(channel="start/track/invalid-edge/branch/end")),
    )

    def test_invalid(self, channel):
        with pytest.raises(RuntimeError):
            channels.Channel(channel)
