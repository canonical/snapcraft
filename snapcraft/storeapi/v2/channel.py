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

from typing import Any, Dict, Optional

from . import progressive

"""
This module holds representations for channels from a Channel Map
from the v2 API provided by the Snap Store.

The full API is documented on https://dashboard.snapcraft.io/docs/v2/en/snaps.html
"""


class MappedChannel:
    """
    Represents a Channel from a channel-map structure from the Snap Store.
    """

    def __repr__(self) -> str:
        return f"<MappedChannel: {self.channel!r} for revision {self.revision!r} and architecture {self.architecture!r}>"

    def __init__(self, payload: Dict[str, Any]) -> None:
        self._payload = payload

    @property
    def channel(self) -> str:
        """Return the channel name."""
        return self._payload["channel"]

    @property
    def revision(self) -> int:
        """Return the revision for this channel."""
        return self._payload["revision"]

    @property
    def architecture(self) -> str:
        """Return the architecture string set for this channel."""
        return self._payload["architecture"]

    @property
    def expiration_date(self) -> Optional[str]:
        """Return the expiration date for this channel."""
        return self._payload["expiration-date"]

    @property
    def progressive(self):
        """Return the progressive release status for this channel."""
        return progressive.Progressive(self._payload["progressive"])


class SnapChannel:
    """
    Represents a Channel from a snap structure from the Snap Store.
    """

    def __repr__(self) -> str:
        return f"<SnapChannel: {self.name!r}>"

    def __init__(self, payload: Dict[str, Any]) -> None:
        self._payload = payload

    @property
    def name(self) -> str:
        """Return the channel name."""
        return self._payload["name"]

    @property
    def track(self) -> str:
        """Return the track for this channel."""
        return self._payload["track"]

    @property
    def risk(self) -> str:
        """Return the risk for this channel."""
        return self._payload["risk"]

    @property
    def branch(self) -> Optional[str]:
        """Return the branch for this channel."""
        return self._payload["branch"]

    @property
    def fallback(self) -> str:
        """Return the fallback for this channel."""
        return self._payload["fallback"]


class Track:
    """Represents a Track structure from the Snap Store."""

    def __repr__(self) -> str:
        return f"<Track: {self.name!r}>"

    def __init__(self, payload: Dict[str, Any]) -> None:
        self._payload = payload

    @property
    def name(self) -> str:
        """Return the track name."""
        return self._payload["name"]

    @property
    def version_pattern(self) -> Optional[str]:
        """Return the version pattern accepted by this track."""
        return self._payload["version-pattern"]
