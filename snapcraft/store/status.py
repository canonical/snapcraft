# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2026 Canonical Ltd
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

"""Representations for results from the v1 status API provided by the Snap Store."""

from __future__ import annotations

from typing import Any

from .channels import Channel
from .constants import DEFAULT_SERIES
from .errors import ChannelNotAvailableOnArchError, InvalidChannelSet


class SnapStatusChannelDetails:
    """Details for a single channel/arch entry from the v1 status API."""

    def __repr__(self) -> str:
        return f"<SnapStatusChannelDetails: {self._snap_name} on {self._arch}>"

    def __iter__(self):
        for attr in ["arch", "revision", "version"]:
            yield getattr(self, attr)

    def __init__(self, *, snap_name: str, arch: str, payload: dict[str, Any]) -> None:
        self._snap_name = snap_name
        self._arch = arch
        self._payload = payload

    @property
    def arch(self) -> str:
        return self._arch

    @property
    def channel(self) -> str:
        return self._payload["channel"]

    @property
    def info(self) -> str:
        return self._payload["info"]

    @property
    def version(self) -> str | None:
        return self._payload.get("version")

    @property
    def revision(self) -> int | None:
        return self._payload.get("revision")


class SnapStatusTrackDetails:
    """Details for a single track from the v1 status API."""

    def __repr__(self) -> str:
        return f"<SnapStatusTrackDetails: {self._snap_name} on {self.track}>"

    def __init__(self, *, snap_name: str, track: str, payload: dict[str, Any]) -> None:
        self._snap_name = snap_name
        self._track = track
        self._payload = payload
        self._arch_channels: dict[str, list[SnapStatusChannelDetails]] = {}

    @property
    def track(self) -> str:
        return self._track

    def get_arches(self) -> list[str]:
        return list(self._payload.keys())

    def get_channel(
        self, *, risk: str, arch: str, branch: str | None = None
    ) -> SnapStatusChannelDetails:
        if arch not in self._arch_channels:
            try:
                self._arch_channels[arch] = [
                    SnapStatusChannelDetails(
                        snap_name=self._snap_name, arch=arch, payload=c
                    )
                    for c in self._payload[arch]
                ]
            except KeyError:
                channel = Channel.from_channel_tuple(
                    track=self.track, risk=risk, branch=branch
                )
                raise ChannelNotAvailableOnArchError(
                    snap_name=self._snap_name,
                    channel=channel,
                    arch=arch,
                )

        channel_key = f"{risk}/{branch}" if branch else risk
        matches = [c for c in self._arch_channels[arch] if channel_key == c.channel]

        try:
            return matches[0]
        except IndexError:
            channel = Channel.from_channel_tuple(
                track=self.track, risk=risk, branch=branch
            )
            raise ChannelNotAvailableOnArchError(
                snap_name=self._snap_name,
                channel=channel,
                arch=arch,
            )


class SnapStatus:
    """Wrap the payload returned by the v1 snap status API endpoint."""

    def __repr__(self) -> str:
        return f"<SnapStatus: {self._snap_name}>"

    def __init__(self, *, snap_name: str, payload: dict[str, Any]) -> None:
        self._snap_name = snap_name
        self._payload = payload

    def get_tracks(self) -> list[str]:
        return list(self._payload["channel_map_tree"].keys())

    def get_track(
        self, track: str, *, device_series: str = DEFAULT_SERIES
    ) -> SnapStatusTrackDetails:
        return SnapStatusTrackDetails(
            snap_name=self._snap_name,
            track=track,
            payload=self._payload["channel_map_tree"][track][device_series],
        )

    def get_channel_set(self, channel: Channel) -> set[SnapStatusChannelDetails]:
        """Return the full build set for a channel (one entry per arch).

        :raises InvalidChannelSet: if any architecture has no released revision
            since promoting a partial build set is not allowed.
        """
        track = self.get_track(channel.track)
        channel_selection = [
            track.get_channel(risk=channel.risk, branch=channel.branch, arch=arch)
            for arch in track.get_arches()
        ]

        # Every arch must have a real revision (a partial build set cannot be promoted).
        missing = [c for c in channel_selection if c.revision is None]
        if missing:
            raise InvalidChannelSet(
                snap_name=self._snap_name,
                channel=channel,
                channel_outliers=missing,
            )

        return set(channel_selection)
