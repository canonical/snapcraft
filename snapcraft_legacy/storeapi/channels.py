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

from typing import Optional, Type

_VALID_RISKS = ["stable", "candidate", "beta", "edge"]


class Channel:
    def __repr__(self) -> str:
        return "{!r}".format(self._channel)

    def __str__(self) -> str:
        return self._channel

    def __eq__(self, other) -> bool:
        return (
            self.track == other.track
            and self.risk == other.risk
            and self.branch == other.branch
        )

    @classmethod
    def from_channel_tuple(
        cls: Type["Channel"], *, track: str, risk: str, branch: Optional[str]
    ) -> "Channel":
        if track and risk and branch:
            channel = "{}/{}/{}".format(track, risk, branch)
        elif track and risk:
            channel = "{}/{}".format(track, risk)
        elif risk and branch:
            channel = "{}/{}".format(risk, branch)
        elif risk:
            channel = risk
        else:
            raise RuntimeError(
                "Incorrect channel tuple {}/{}/{}.".format(track, risk, branch)
            )

        return cls(channel)

    def __init__(self, channel: str) -> None:
        channel_parts = channel.split("/")
        if len(channel_parts) == 1 and channel_parts[0] in _VALID_RISKS:
            self._track = None
            self._risk = channel_parts[0]
            self._branch = None
        elif len(channel_parts) == 3 and channel_parts[1] in _VALID_RISKS:
            self._track = channel_parts[0]
            self._risk = channel_parts[1]
            self._branch = channel_parts[2]
        elif len(channel_parts) == 2 and channel_parts[0] in _VALID_RISKS:
            self._track = None
            self._risk = channel_parts[0]
            self._branch = channel_parts[1]
        elif len(channel_parts) == 2 and channel_parts[1] in _VALID_RISKS:
            self._track = channel_parts[0]
            self._risk = channel_parts[1]
            self._branch = None
        else:
            raise RuntimeError("Channel logic failed for: {!r}".format(channel))
        self._channel = channel

    @property
    def track(self) -> str:
        if self._track is None:
            return "latest"
        else:
            return self._track

    @property
    def risk(self) -> str:
        return self._risk

    @property
    def branch(self) -> Optional[str]:
        return self._branch
