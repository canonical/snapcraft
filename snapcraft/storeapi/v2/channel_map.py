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

import jsonschema

from ._api_schema import CHANNEL_MAP_JSONSCHEMA

"""
This module holds representations for results for the v2 channel-map
API endpoint provided by the Snap Store.

The full API is documented on
https://dashboard.snapcraft.io/docs/v2/en/snaps.html#snap-channel-map
"""


class Progressive:
    """Represent Progressive information for a MappedChannel."""

    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "Progressive":
        jsonschema.validate(
            payload,
            CHANNEL_MAP_JSONSCHEMA["properties"]["channel-map"]["items"]["properties"][
                "progressive"
            ],
        )
        return cls(
            paused=payload["paused"],
            percentage=payload["percentage"],
            current_percentage=payload["current-percentage"],
        )

    def marshal(self) -> Dict[str, Any]:
        return {
            "paused": self.paused,
            "percentage": self.percentage,
            "current-percentage": self.current_percentage,
        }

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__}: {self.current_percentage!r}=>{self.percentage!r}>"

    def __init__(
        self,
        *,
        paused: Optional[bool],
        percentage: Optional[float],
        current_percentage: Optional[float],
    ) -> None:
        self.paused = paused
        self.percentage = percentage
        self.current_percentage = current_percentage


class MappedChannel:
    """Represent a mapped channel item for "channel-map" in ChannelMap."""

    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "MappedChannel":
        jsonschema.validate(
            payload, CHANNEL_MAP_JSONSCHEMA["properties"]["channel-map"]["items"]
        )
        return cls(
            channel=payload["channel"],
            revision=payload["revision"],
            architecture=payload["architecture"],
            expiration_date=payload["expiration-date"],
            progressive=Progressive.unmarshal(payload["progressive"]),
        )

    def marshal(self) -> Dict[str, Any]:
        return {
            "channel": self.channel,
            "revision": self.revision,
            "architecture": self.architecture,
            "expiration-date": self.expiration_date,
            "progressive": self.progressive.marshal(),
        }

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__}: {self.channel!r} for revision {self.revision!r} and architecture {self.architecture!r}>"

    def __init__(
        self,
        *,
        channel: str,
        revision: int,
        architecture: str,
        expiration_date: Optional[str],
        progressive: Progressive,
    ) -> None:
        self.channel = channel
        self.revision = revision
        self.architecture = architecture
        self.expiration_date = expiration_date
        self.progressive = progressive


class Revision:
    """Represent a revision item for "revisions" in ChannelMap."""

    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "Revision":
        jsonschema.validate(
            payload, CHANNEL_MAP_JSONSCHEMA["properties"]["revisions"]["items"]
        )
        return cls(
            revision=payload["revision"],
            version=payload["version"],
            architectures=payload["architectures"],
        )

    def marshal(self) -> Dict[str, Any]:
        return {
            "revision": self.revision,
            "version": self.version,
            "architectures": self.architectures,
        }

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__}: {self.revision!r} for version {self.version!r} and architectures {self.architectures!r}>"

    def __init__(
        self, *, revision: int, version: str, architectures: List[str]
    ) -> None:
        self.revision = revision
        self.version = version
        self.architectures = architectures


class SnapChannel:
    """Represent a channel item in "channels" in Snap."""

    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "SnapChannel":
        jsonschema.validate(
            payload,
            CHANNEL_MAP_JSONSCHEMA["properties"]["snap"]["properties"]["channels"][
                "items"
            ],
        )
        return cls(
            name=payload["name"],
            track=payload["track"],
            risk=payload["risk"],
            branch=payload["branch"],
            fallback=payload["fallback"],
        )

    def marshal(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "track": self.track,
            "risk": self.risk,
            "branch": self.branch,
            "fallback": self.fallback,
        }

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__}: {self.name!r}>"

    def __init__(
        self,
        *,
        name: str,
        track: str,
        risk: str,
        branch: Optional[str],
        fallback: Optional[str],
    ) -> None:
        self.name = name
        self.track = track
        self.risk = risk
        self.branch = branch
        self.fallback = fallback


class SnapTrack:
    """Represent a track item in "tracks" in Snap."""

    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "SnapTrack":
        jsonschema.validate(
            payload,
            CHANNEL_MAP_JSONSCHEMA["properties"]["snap"]["properties"]["tracks"][
                "items"
            ],
        )
        return cls(
            name=payload["name"],
            status=payload["status"],
            creation_date=payload["creation-date"],
            version_pattern=payload["version-pattern"],
        )

    def marshal(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "status": self.status,
            "creation-date": self.creation_date,
            "version-pattern": self.version_pattern,
        }

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__}: {self.name!r}>"

    def __init__(
        self,
        *,
        name: str,
        status: str,
        creation_date: Optional[str],
        version_pattern: Optional[str],
    ) -> None:
        self.name = name
        self.status = status
        self.creation_date = creation_date
        self.version_pattern = version_pattern


class Snap:
    """Represent "snap" in ChannelMap."""

    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "Snap":
        jsonschema.validate(payload, CHANNEL_MAP_JSONSCHEMA["properties"]["snap"])
        return cls(
            name=payload["name"],
            channels=[SnapChannel.unmarshal(sc) for sc in payload["channels"]],
            tracks=[SnapTrack.unmarshal(st) for st in payload["tracks"]],
        )

    def marshal(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "channels": [sc.marshal() for sc in self.channels],
            "tracks": [st.marshal() for st in self.tracks],
        }

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__}: {self.name!r}>"

    def __init__(
        self, *, name: str, channels: List[SnapChannel], tracks: List[SnapTrack]
    ) -> None:
        self.name = name
        self.channels = channels
        self.tracks = tracks


class ChannelMap:
    """Represent the data returned from the channel-map Snap Store endpoint."""

    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "ChannelMap":
        jsonschema.validate(payload, CHANNEL_MAP_JSONSCHEMA)
        return cls(
            channel_map=[MappedChannel.unmarshal(c) for c in payload["channel-map"]],
            revisions=[Revision.unmarshal(r) for r in payload["revisions"]],
            snap=Snap.unmarshal(payload["snap"]),
        )

    def marshal(self) -> Dict[str, Any]:
        return {
            "channel-map": [c.marshal() for c in self.channel_map],
            "revisions": [r.marshal() for r in self.revisions],
            "snap": self.snap.marshal(),
        }

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__}: {self.snap.name!r}>"

    def __init__(
        self, *, channel_map: List[MappedChannel], revisions: List[Revision], snap: Snap
    ) -> None:
        self.channel_map = channel_map
        self.revisions = revisions
        self.snap = snap

    def get_mapped_channel(
        self, *, channel_name: str, architecture: str, progressive: bool
    ) -> MappedChannel:
        channels_with_name = (
            cm for cm in self.channel_map if cm.channel == channel_name
        )
        channels_with_arch = (
            cm for cm in channels_with_name if cm.architecture == architecture
        )

        if progressive:
            channels = [
                cm for cm in channels_with_arch if cm.progressive.percentage is not None
            ]
        else:
            channels = [
                cm for cm in channels_with_arch if cm.progressive.percentage is None
            ]

        try:
            return channels[0]
        except IndexError:
            raise ValueError(
                f"No channel mapped to {channel_name!r} for architecture {architecture!r} when progressive is {progressive!r}"
            )

    def get_channel_info(self, channel_name: str) -> SnapChannel:
        for snap_channel in self.snap.channels:
            if snap_channel.name == channel_name:
                return snap_channel
        raise ValueError(f"No channel information for {channel_name!r}")

    def get_revision(self, revision_number: int) -> Revision:
        for revision_item in self.revisions:
            if revision_item.revision == revision_number:
                return revision_item
        raise ValueError(f"No revision information for {revision_number!r}")

    def get_existing_architectures(self) -> Set[str]:
        architectures: List[str] = list()
        for revision_item in self.revisions:
            architectures.extend(revision_item.architectures)

        return set(architectures)
