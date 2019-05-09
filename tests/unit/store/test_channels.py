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

from testtools.matchers import Equals

from snapcraft.storeapi import channels
from tests import unit


class ChannelTest(unit.TestCase):
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

    def test_channel(self):
        c = channels.Channel(self.channel)
        self.assertThat(c.track, Equals(self.expected_track))
        self.assertThat(c.risk, Equals(self.expected_risk))
        self.assertThat(c.branch, Equals(self.expected_branch))
        self.assertThat(str(c), Equals(self.expected_channel))
        self.assertThat(repr(c), Equals("'{}'".format(self.expected_channel)))


class ChannelTupleTest(unit.TestCase):
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

    def test_channel(self):
        c = channels.Channel.from_channel_tuple(
            track=self.track, risk=self.risk, branch=self.branch
        )
        self.assertThat(c.track, Equals(self.expected_track))
        self.assertThat(c.risk, Equals(self.risk))
        self.assertThat(c.branch, Equals(self.branch))
        self.assertThat(str(c), Equals(self.expected_channel))
        self.assertThat(repr(c), Equals("'{}'".format(self.expected_channel)))


class InvalidChannelTest(unit.TestCase):
    scenarios = (
        ("invalid risk", dict(channel="invalid-edge")),
        ("invalid track/risk", dict(channel="track/invalid-edge")),
        ("invalid track/risk/branch", dict(channel="track/invalid-edge/branch")),
        ("invalid tuple", dict(channel="start/track/invalid-edge/branch/end")),
    )

    def test_invalid(self):
        self.assertRaises(RuntimeError, channels.Channel, self.channel)
