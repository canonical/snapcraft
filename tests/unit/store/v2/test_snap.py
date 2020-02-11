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

from testtools.matchers import Equals, Is, IsInstance, HasLength

from snapcraft.storeapi.v2 import channel, snap
from tests import unit


class SnapTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.payload = {
            "name": "my-snap",
            "private": True,
            "default-track": None,
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
                {"name": "latest", "version-pattern": None},
                {"name": "2.0", "version-pattern": "2\\..*"},
            ],
        }

    def expect_common(self, s):
        self.expectThat(repr(s), Equals("<Snap: 'my-snap'>"))
        self.expectThat(s.name, Equals(self.payload["name"]))
        self.expectThat(s.private, Equals(self.payload["private"]))

        snap_channels = s.channels
        self.expectThat(snap_channels, HasLength(2))
        self.expectThat(snap_channels[0], IsInstance(channel.SnapChannel))
        self.expectThat(snap_channels[1], IsInstance(channel.SnapChannel))

        snap_tracks = s.tracks
        self.expectThat(snap_tracks, HasLength(2))
        self.expectThat(snap_tracks[0], IsInstance(channel.Track))
        self.expectThat(snap_tracks[1], IsInstance(channel.Track))

    def test_snap(self):
        s = snap.Snap(payload=self.payload)

        self.expect_common(s)
        self.expectThat(s.default_track, Is(None))

    def test_snap_default_track(self):
        self.payload.update({"default-track": "latest"})

        s = snap.Snap(payload=self.payload)

        self.expect_common(s)
        self.expectThat(s.default_track, Equals("latest"))
