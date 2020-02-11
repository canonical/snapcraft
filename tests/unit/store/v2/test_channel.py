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

from testtools.matchers import Equals, Is, IsInstance

from snapcraft.storeapi.v2 import channel, progressive
from tests import unit


class MappedChannelTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.payload = {
            "architecture": "amd64",
            "channel": "latest/stable",
            "expiration-date": None,
            "progressive": {"key": None, "paused": None, "percentage": None},
            "revision": 2,
        }

    def test_channel(self):
        mc = channel.MappedChannel(payload=self.payload)

        self.expectThat(
            repr(mc),
            Equals(
                "<MappedChannel: 'latest/stable' for revision 2 and architecture 'amd64'>"
            ),
        )
        self.expectThat(mc.channel, Equals(self.payload["channel"]))
        self.expectThat(mc.revision, Equals(self.payload["revision"]))
        self.expectThat(mc.architecture, Equals(self.payload["architecture"]))
        self.expectThat(mc.progressive, IsInstance(progressive.Progressive))
        self.expectThat(mc.expiration_date, Is(None))

    def test_channel_with_expiration(self):
        date = "2020-02-11T17:51:40.891996Z"
        self.payload.update({"expiration-date": date})

        mc = channel.MappedChannel(payload=self.payload)

        self.expectThat(
            repr(mc),
            Equals(
                "<MappedChannel: 'latest/stable' for revision 2 and architecture 'amd64'>"
            ),
        )
        self.expectThat(mc.channel, Equals(self.payload["channel"]))
        self.expectThat(mc.revision, Equals(self.payload["revision"]))
        self.expectThat(mc.architecture, Equals(self.payload["architecture"]))
        self.expectThat(mc.progressive, IsInstance(progressive.Progressive))
        self.expectThat(mc.expiration_date, Equals(date))


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
        sc = channel.SnapChannel(payload=self.payload)

        self.expectThat(repr(sc), Equals("<SnapChannel: 'latest/candidate'>"))
        self.expectThat(sc.name, Equals(self.payload["name"]))
        self.expectThat(sc.track, Equals(self.payload["track"]))
        self.expectThat(sc.risk, Equals(self.payload["risk"]))
        self.expectThat(sc.branch, Is(None)),
        self.expectThat(sc.fallback, Is(None)),

    def test_channel_with_branch(self):
        self.payload.update({"branch": "test-branch"})

        sc = channel.SnapChannel(payload=self.payload)

        self.expectThat(repr(sc), Equals("<SnapChannel: 'latest/candidate'>"))
        self.expectThat(sc.name, Equals(self.payload["name"]))
        self.expectThat(sc.track, Equals(self.payload["track"]))
        self.expectThat(sc.risk, Equals(self.payload["risk"]))
        self.expectThat(sc.branch, Equals(self.payload["branch"]))
        self.expectThat(sc.fallback, Is(None)),

    def test_channel_with_fallback(self):
        self.payload.update({"fallback": "latest/stable"})

        sc = channel.SnapChannel(payload=self.payload)

        self.expectThat(repr(sc), Equals("<SnapChannel: 'latest/candidate'>"))
        self.expectThat(sc.name, Equals(self.payload["name"]))
        self.expectThat(sc.track, Equals(self.payload["track"]))
        self.expectThat(sc.risk, Equals(self.payload["risk"]))
        self.expectThat(sc.branch, Is(None)),
        self.expectThat(sc.fallback, Equals(self.payload["fallback"]))


class TrackTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.payload = {"name": "latest", "version-pattern": None}

    def test_track(self):
        t = channel.Track(payload=self.payload)

        self.expectThat(repr(t), Equals("<Track: 'latest'>"))
        self.expectThat(t.name, Equals(self.payload["name"]))
        self.expectThat(t.version_pattern, Is(None)),

    def test_with_version_pattern(self):
        self.payload.update({"version-pattern": "0\\.*"})

        t = channel.Track(payload=self.payload)

        self.expectThat(repr(t), Equals("<Track: 'latest'>"))
        self.expectThat(t.name, Equals(self.payload["name"]))
        self.expectThat(t.version_pattern, Equals(self.payload["version-pattern"]))
