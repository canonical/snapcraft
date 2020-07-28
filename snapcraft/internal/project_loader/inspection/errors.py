# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

from snapcraft.internal.errors import SnapcraftException


class SnapcraftNoSuchFileError(SnapcraftException):
    def __init__(self, *, path: str) -> None:
        self.path = path

    def get_brief(self) -> str:
        return f"Failed to find part that provided path {self.path!r}: file does not exist."

    def get_resolution(self) -> str:
        return "Check the file path and try again."


class SnapcraftInspectError(SnapcraftException):
    # Use a different exit code for these errors so the orchestrating snapcraft can
    # differentiate them.
    def get_exit_code(self):
        return 3


class SnapcraftNoStepsRunError(SnapcraftInspectError):
    def get_brief(self) -> str:
        return "Failed to get latest step: no steps have run"

    def get_resolution(self) -> str:
        return "Run 'snapcraft clean' and retry build."


class SnapcraftProvidesInvalidFilePathError(SnapcraftInspectError):
    def __init__(self, *, path: str) -> None:
        self.path = path

    def get_brief(self) -> str:
        return f"Failed to find part that provided path {self.path!r}: file is not in the staging or priming area."

    def get_resolution(self) -> str:
        return "Ensure the path is in the staging or priming area and try again."


class SnapcraftUntrackedFileError(SnapcraftInspectError):
    def __init__(self, *, path: str) -> None:
        self.path = path

    def get_brief(self) -> str:
        return f"Failed to find part that provided path {self.path!r}: it may have been provided by a scriplet."

    def get_resolution(self) -> str:
        return "Run 'snapcraft clean' and retry build."
