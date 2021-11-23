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

from typing import Any, Dict, List, Optional, Set

from . import channels, constants, errors

# FIXME Obtain link to docs
"""
This module holds representations for results from the status API provided by
the Snap Store.
"""


class SnapStatusChannelDetails:
    def __repr__(self) -> str:
        return "<SnapStatusChannelDetails: {} on {}>".format(
            self._snap_name, self._arch
        )

    def __iter__(self):
        for attr in ["arch", "revision", "version"]:
            yield self.__getattribute__(attr)

    def __init__(self, *, snap_name: str, arch: str, payload: Dict[str, Any]) -> None:
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
    def version(self) -> Optional[str]:
        return self._payload.get("version")

    @property
    def revision(self) -> Optional[int]:
        return self._payload.get("revision")


class SnapStatusTrackDetails:
    def __repr__(self) -> str:
        return "<SnapStatusTrackDetails: {} on {}>".format(self._snap_name, self.track)

    def __init__(self, *, snap_name: str, track: str, payload: Dict[str, Any]) -> None:
        self._snap_name = snap_name
        self._track = track
        self._payload = payload
        self._arch_channels = dict()  # type: Dict[str, List[SnapStatusChannelDetails]]

    @property
    def track(self) -> str:
        return self._track

    def get_arches(self) -> List[str]:
        return [k for k in self._payload.keys()]

    def get_channel(
        self, *, risk: str, arch: str, branch: Optional[str] = None
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
                raise errors.ChannelNotAvailableOnArchError(
                    snap_name=self._snap_name,
                    channel=channels.Channel.from_channel_tuple(
                        track=self.track, risk=risk, branch=branch
                    ),
                    arch=arch,
                )

        if branch:
            channel = "{}/{}".format(risk, branch)
        else:
            channel = risk

        c = [c for c in self._arch_channels[arch] if channel == c.channel]
        # We assume the API will not return duplicate mappings
        try:
            return c[0]
        except IndexError:
            raise errors.ChannelNotAvailableOnArchError(
                snap_name=self._snap_name,
                channel=channels.Channel.from_channel_tuple(
                    track=self.track, risk=risk, branch=branch
                ),
                arch=arch,
            )


class SnapStatus:
    def __repr__(self) -> str:
        return "<SnapStatus: {}>".format(self._snap_name)

    def __init__(self, *, snap_name: str, payload: Dict[str, Any]) -> None:
        self._snap_name = snap_name
        self._payload = payload

    def get_tracks(self) -> List[str]:
        return [k for k in self._payload["channel_map_tree"].keys()]

    def get_track(
        self, track: str, *, device_series=constants.DEFAULT_SERIES
    ) -> SnapStatusTrackDetails:
        return SnapStatusTrackDetails(
            snap_name=self._snap_name,
            track=track,
            payload=self._payload["channel_map_tree"][track][device_series],
        )

    def get_channel_set(
        self, channel: channels.Channel
    ) -> Set[SnapStatusChannelDetails]:
        track = self.get_track(channel.track)
        channel_selection = [
            track.get_channel(risk=channel.risk, branch=channel.branch, arch=arch)
            for arch in track.get_arches()
        ]

        # Make sure this is a valid build set (has released revisions for every channel/arch)
        channel_outliers = [c for c in channel_selection if c.revision is None]
        if channel_outliers:
            raise errors.InvalidChannelSet(
                snap_name=self._snap_name,
                channel=channel,
                channel_outliers=channel_outliers,
            )

        return set(channel_selection)
