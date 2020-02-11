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

"""
This module holds representations for results for Progressive Releases
from the v2 releases API provided by the Snap Store.

The full API is documented on https://dashboard.snapcraft.io/docs/v2/en/snaps.html
"""


class Progressive:
    """Represents a progressive release structure from the Snap Store."""

    def __repr__(self) -> str:
        return f"<Progressive: {self.key!r}>"

    def __init__(self, payload: Dict[str, Any]) -> None:
        self._payload = payload

    @property
    def key(self) -> Optional[str]:
        """
        Return the progressive key.

        None shall be returned if not part of a progressive release.
        """
        return self._payload["key"]

    @property
    def paused(self) -> Optional[bool]:
        """
        Return True if the progressive release is paused.

        None shall be returned if not part of a progressive release.
        """
        return self._payload["paused"]

    @property
    def percentage(self) -> Optional[float]:
        """
        Return the progressive percentage.

        None shall be returned if not part of a progressive release.
        """
        return self._payload["percentage"]
