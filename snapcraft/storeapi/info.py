# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019-2020 Canonical Ltd
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

import os
from typing import Any, Dict, List, Optional

from snapcraft.file_utils import calculate_hash

from . import errors

"""
This module holds representations for results from the v2 info API provided by
the Snap Store.

The full API is documented on https://api.snapcraft.io/api-docs/info.html
"""

# FIXME: attributes are manually created to keep the version of mypy we are using happy.
#        This might be a good thing in the long run.


class SnapChannelDetails:
    def __repr__(self) -> str:
        return "<SnapChannelDetails: {!r} for arch {!r} released at {!r}>".format(
            self.name, self.architecture, self.released_at
        )

    def __init__(self, details: Dict[str, Any]) -> None:
        self._payload = details

    @property
    def architecture(self) -> str:
        return self._payload["architecture"]

    @property
    def name(self) -> str:
        return self._payload["name"]

    @property
    def released_at(self) -> str:
        return self._payload["released-at"]

    @property
    def risk(self) -> str:
        return self._payload["risk"]

    @property
    def track(self) -> str:
        return self._payload["track"]


class SnapDownloadDetails:
    def __repr__(self) -> str:
        return "<SnapDownloadDetails: {!r} with size {!r}>".format(self.url, self.size)

    def __init__(self, download: Dict[str, Any]) -> None:
        self._payload = download

    @property
    def url(self) -> str:
        return self._payload["url"]

    @property
    def size(self) -> int:
        return self._payload["size"]

    @property
    def sha3_384(self) -> str:
        return self._payload["sha3-384"]

    def verify(self, path: str) -> None:
        if not os.path.exists(path):
            raise errors.DownloadNotFoundError(path=path)

        calculated_hash = calculate_hash(path, algorithm="sha3_384")
        if self.sha3_384 != calculated_hash:
            raise errors.SHAMismatchError(
                path=path, expected=self.sha3_384, calculated=calculated_hash
            )


class SnapPublisherDetails:
    def __repr__(self) -> str:
        return "<SnapPublisherDetails: {!r}>".format(self.username)

    def __init__(self, publisher: Dict[str, Any]) -> None:
        self._payload = publisher

    @property
    def id(self) -> str:
        return self._payload["id"]

    @property
    def display_name(self) -> str:
        return self._payload["display-name"]

    @property
    def username(self) -> str:
        return self._payload["username"]

    @property
    def validation(self) -> str:
        return self._payload["validation"]


class SnapDetails:
    def __repr__(self) -> str:
        return "<SnapDetails: {!r} with snap-id {!r}>".format(self.name, self.snap_id)

    def __init__(self, snap: Dict[str, Any]) -> None:
        self._payload = snap
        self._publisher = None  # type: Optional[SnapPublisherDetails]

    @property
    def publisher(self) -> SnapPublisherDetails:
        if self._publisher is None:
            publisher = self._payload.get("publisher")
            if publisher is None:
                # Shouldn't happen, but raise an error if it does.
                raise RuntimeError(f"no publisher found for snap {self.name!r}")

            self._publisher = SnapPublisherDetails(publisher)
        return self._publisher

    @property
    def name(self) -> str:
        return self._payload["name"]

    @property
    def snap_id(self) -> str:
        return self._payload["snap-id"]


class SnapChannelMapping:
    def __repr__(self) -> str:
        return "<SnapChannelMapping: {!r} on {!r}>".format(
            self.channel_details, self.revision
        )

    def __init__(self, channel: Dict[str, Any]) -> None:
        self._payload = channel
        self._channel_details = None  # type: Optional[SnapChannelDetails]
        self._download = None  # type: Optional[SnapDownloadDetails]

    @property
    def channel_details(self) -> SnapChannelDetails:
        if self._channel_details is None:
            channel = self._payload.get("channel")
            if channel is None:
                # Shouldn't happen, but raise an error if it does.
                raise RuntimeError(f"no channel found for {self._payload!r}")

            self._channel_details = SnapChannelDetails(channel)
        return self._channel_details

    @property
    def download(self) -> SnapDownloadDetails:
        if self._download is None:
            download = self._payload.get("download")
            if download is None:
                # Shouldn't happen, but raise an error if it does.
                raise RuntimeError(f"no download found for {self._payload!r}")

            self._download = SnapDownloadDetails(download)
        return self._download

    @property
    def revision(self) -> int:
        return self._payload["revision"]

    @property
    def confinement(self) -> str:
        return self._payload["confinement"]

    @property
    def version(self) -> str:
        return self._payload["version"]


class SnapInfo:
    def __repr__(self) -> str:
        return "<SnapInfo: {!r}>".format(self.name)

    def __init__(self, snap_info_resp: Dict[str, Any]) -> None:
        self._payload = snap_info_resp
        self._channel_map = None  # type: Optional[List[SnapChannelMapping]]
        self._snap = None  # type: Optional[SnapDetails]

    @property
    def channel_map(self) -> List[SnapChannelMapping]:
        if self._channel_map is None:
            self._channel_map = [
                SnapChannelMapping(i) for i in self._payload.get("channel-map", [])
            ]
        return self._channel_map

    @property
    def snap(self) -> SnapDetails:
        if self._snap is None:
            snap = self._payload.get("snap")
            if snap is None:
                # Shouldn't happen, but raise an error if it does.
                raise RuntimeError(f"no snap found for {self._payload!r}")

            self._snap = SnapDetails(snap)
        return self._snap

    @property
    def name(self) -> str:
        return self._payload["name"]

    @property
    def snap_id(self) -> str:
        return self._payload["snap-id"]

    def get_channel_mapping(
        self, *, risk: str, arch: Optional[str] = None, track: Optional[str] = None
    ) -> SnapChannelMapping:
        if track is None:
            track_filter = "latest"
        else:
            track_filter = track

        arch_match = (
            c
            for c in self.channel_map
            if arch is None or c.channel_details.architecture == arch
        )
        track_match = (c for c in arch_match if c.channel_details.track == track_filter)
        risk_match = [c for c in track_match if c.channel_details.risk == risk]

        if not risk_match:
            channel = "{}/{}".format(track, risk) if track else risk
            raise errors.SnapNotFoundError(snap_name=self.name, channel=channel)

        # We assume the API will not return duplicate mappings
        return risk_match[0]
