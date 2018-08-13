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

import snapcraft.internal.errors


class NoSuchFileError(snapcraft.internal.errors.SnapcraftError):

    fmt = (
        "Failed to find part that provided path: {path!r} does not "
        "exist.\n"
        "Check the file path and try again."
    )

    def __init__(self, path):
        super().__init__(path=path)


class SnapcraftInspectError(snapcraft.internal.errors.SnapcraftError):
    # Use a different exit code for these errors so the orchestrating snapcraft can
    # differentiate them.
    def get_exit_code(self):
        return 3


class ProvidesInvalidFilePathError(SnapcraftInspectError):

    fmt = (
        "Failed to find part that provides path: {path!r} is not in the "
        "staging or priming area.\n"
        "Ensure the path is in the staging or priming area and try again."
    )

    def __init__(self, path):
        super().__init__(path=path)


class UntrackedFileError(SnapcraftInspectError):

    fmt = "No known parts provided {path!r}. It may have been provided by a scriptlet."

    def __init__(self, path):
        super().__init__(path=path)


class NoStepsRunError(SnapcraftInspectError):
    fmt = "Failed to get latest step: no steps have run"
