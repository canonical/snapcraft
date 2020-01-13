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

from typing import Any, Dict, List, Optional, Set

from . import channel, revision, snap

"""
This module holds representations for results for a Snap Channel Map
from the v2 releases API provided by the Snap Store.

The full API is documented on https://dashboard.snapcraft.io/docs/v2/en/snaps.html
"""


class SnapChannelMap:
    """Represents the data returned from the channel-map call from the Snap Store."""

    def __repr__(self) -> str:
        return "<SnapChannelMap: {!r}>".format(self.snap.name)

    def __init__(self, snap_channel_map: Dict[str, Any]) -> None:
        self._payload = snap_channel_map
        self._channelmap: Optional[List[channel.MappedChannel]] = None
        self._revisions: Optional[List[revision.Revision]] = None
        self._snap: Optional[snap.Snap] = None

    @property
    def channel_map(self) -> List[channel.MappedChannel]:
        if self._channelmap is not None:
            return self._channelmap

        self._channelmap = [
            channel.MappedChannel(c) for c in self._payload["channel-map"]
        ]
        return self._channelmap

    @property
    def revisions(self) -> List[revision.Revision]:
        if self._revisions is not None:
            return self._revisions

        self._revisions = [revision.Revision(r) for r in self._payload["revisions"]]
        return self._revisions

    @property
    def snap(self):
        if self._snap is not None:
            return self._snap

        self._snap = snap.Snap(self._payload["snap"])
        return self._snap

    def get_mapped_channel(
        self, channel_name: str, architecture: str
    ) -> channel.MappedChannel:
        for channel_mapping in self.channel_map:
            if (
                channel_mapping.channel == channel_name
                and channel_mapping.architecture == architecture
            ):
                return channel_mapping
        raise ValueError(
            f"No channel mapped to {channel_name!r} for architecture {architecture!r}"
        )

    def get_channel_info(self, channel_name: str) -> channel.SnapChannel:
        for snap_channel in self.snap.channels:
            if snap_channel.name == channel_name:
                return snap_channel
        raise ValueError(f"No channel information for {channel_name!r}")

    def get_revision(self, revision_number: int) -> revision.Revision:
        for revision_item in self.revisions:
            if revision_item.revision == revision_number:
                return revision_item
        raise ValueError(f"No revision information for {revision_number!r}")

    def get_existing_architectures(self) -> Set[str]:
        architectures: List[str] = list()
        for revision_item in self.revisions:
            architectures.extend(revision_item.architectures)

        return set(architectures)
