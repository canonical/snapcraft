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

"""Channel parsing and representation for Snap Store channels."""

from __future__ import annotations

from typing_extensions import Self

from snapcraft import errors

_VALID_RISKS = ["stable", "candidate", "beta", "edge"]


class Channel:
    """Parse and represent a Snap Store channel string.

    The format is ``[<track>/]<risk>[/<branch>]`` where risk must be one of
    ``stable``, ``candidate``, ``beta``, or ``edge``.
    """

    def __repr__(self) -> str:
        return repr(self._channel)

    def __str__(self) -> str:
        return self._channel

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, Channel):
            return NotImplemented
        return (
            self.track == other.track
            and self.risk == other.risk
            and self.branch == other.branch
        )

    @classmethod
    def from_channel_tuple(cls, *, track: str, risk: str, branch: str | None) -> Self:
        """Create a Channel from its individual components."""
        if track and risk and branch:
            channel = f"{track}/{risk}/{branch}"
        elif track and risk:
            channel = f"{track}/{risk}"
        elif risk and branch:
            channel = f"{risk}/{branch}"
        elif risk:
            channel = risk
        else:
            raise errors.SnapcraftError(
                f"Incorrect channel tuple {track}/{risk}/{branch}."
            )
        return cls(channel)

    def __init__(self, channel: str) -> None:
        parts = channel.split("/")
        if len(parts) == 1 and parts[0] in _VALID_RISKS:
            self._track: str | None = None
            self._risk = parts[0]
            self._branch: str | None = None
        elif len(parts) == 3 and parts[1] in _VALID_RISKS:
            self._track = parts[0]
            self._risk = parts[1]
            self._branch = parts[2]
        elif len(parts) == 2 and parts[0] in _VALID_RISKS:
            self._track = None
            self._risk = parts[0]
            self._branch = parts[1]
        elif len(parts) == 2 and parts[1] in _VALID_RISKS:
            self._track = parts[0]
            self._risk = parts[1]
            self._branch = None
        else:
            raise errors.SnapcraftError(f"Channel logic failed for: {channel!r}")
        self._channel = channel

    @property
    def track(self) -> str:
        """The track name, defaulting to 'latest' if not specified."""
        return self._track or "latest"

    @property
    def risk(self) -> str:
        """The risk level (stable, candidate, beta, or edge)."""
        return self._risk

    @property
    def branch(self) -> str | None:
        """The branch name, or None if not specified."""
        return self._branch
