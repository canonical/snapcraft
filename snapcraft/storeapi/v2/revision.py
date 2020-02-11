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

from typing import Any, Dict, List

"""
This module holds representations for results for a Revision
from the v2 API for a snap-channel-map provided by the Snap Store.

The full API is documented on https://dashboard.snapcraft.io/docs/v2/en/snaps.html
"""


class Revision:
    def __repr__(self) -> str:
        return f"<Revision: {self.revision!r} for version {self.version!r} and architectures {self.architectures!r}>"

    def __init__(self, payload: Dict[str, Any]) -> None:
        self._payload = payload

    @property
    def revision(self) -> int:
        """Return the revision."""
        return self._payload["revision"]

    @property
    def version(self) -> str:
        """Return the version string set for this revision."""
        return self._payload["version"]

    @property
    def architectures(self) -> List[str]:
        """Return the architectures this revision supports."""
        return self._payload["architectures"]
