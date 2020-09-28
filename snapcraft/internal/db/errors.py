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

import pathlib

from snapcraft.internal.errors import SnapcraftException


class SnapcraftDatastoreVersionUnsupported(SnapcraftException):
    def __init__(
        self, *, path: pathlib.Path, current_version: int, supported_version: int
    ) -> None:
        self.path = path
        self.current_version = current_version
        self.supported_version = supported_version

    def get_brief(self) -> str:
        return f"This version of snapcraft does not support version {self.current_version!r} of the {self.path} datastore."

    def get_resolution(self) -> str:
        return "Use 'snap revert' or 'snap refresh' to install previously used version of snapcraft (or newer)."
