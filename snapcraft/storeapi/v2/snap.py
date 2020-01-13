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

from typing import Any, Dict, List, Optional

from . import channel

"""
This module holds representations for results for a Snap
from the v2 releases API provided by the Snap Store.

The full API is documented on https://dashboard.snapcraft.io/docs/v2/en/snaps.html
"""


class Snap:
    """Represents a Snap structure from the Snap Store."""

    def __repr__(self) -> str:
        return "<Snap: {!r}>".format(self.name)

    def __init__(self, snap: Dict[str, Any]) -> None:
        self._payload = snap
        self._channels: Optional[List[channel.SnapChannel]] = None
        self._tracks: Optional[List[channel.Track]] = None

    @property
    def name(self) -> str:
        return self._payload["name"]

    @property
    def private(self) -> bool:
        return self._payload["private"]

    @property
    def default_track(self) -> Optional[str]:
        return self._payload["default-track"]

    @property
    def channels(self) -> List[channel.SnapChannel]:
        """Return a list of channels for this snap."""
        if self._channels is not None:
            return self._channels

        self._channels = [channel.SnapChannel(t) for t in self._payload["channels"]]
        return self._channels

    @property
    def tracks(self) -> List[channel.Track]:
        """Return a list of tracks for this snap."""
        if self._tracks is not None:
            return self._tracks

        self._tracks = [channel.Track(t) for t in self._payload["tracks"]]
        return self._tracks
