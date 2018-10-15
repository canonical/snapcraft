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

from snapcraft.internal.errors import SnapcraftError


class MissingSnapcraftYamlError(SnapcraftError):

    fmt = (
        "Could not find {snapcraft_yaml_file_path}. Are you sure you are "
        "in the right directory?\n"
        "To start a new project, use `snapcraft init`"
    )

    def __init__(self, *, snapcraft_yaml_file_path):
        super().__init__(snapcraft_yaml_file_path=snapcraft_yaml_file_path)


class YamlValidationError(SnapcraftError):

    fmt = "Issues while validating {source}: {message}"

    def __init__(self, message, source="snapcraft.yaml"):
        super().__init__(message=message, source=source)


class DuplicateSnapcraftYamlError(SnapcraftError):

    fmt = (
        "Found a {snapcraft_yaml_file_path!r} and a "
        "{other_snapcraft_yaml_file_path!r}.\n"
        "Please remove one and try again."
    )

    def __init__(
        self, *, snapcraft_yaml_file_path: str, other_snapcraft_yaml_file_path: str
    ) -> None:
        super().__init__(
            snapcraft_yaml_file_path=snapcraft_yaml_file_path,
            other_snapcraft_yaml_file_path=other_snapcraft_yaml_file_path,
        )


class MultipassMissingNonLinuxError(SnapcraftError):
    fmt = (
        "You need multipass installed to build snaps:\n"
        "https://github.com/CanonicalLtd/multipass/releases"
    )


class MultipassMissingLinuxError(SnapcraftError):
    fmt = (
        "You need multipass installed to build snaps which use the base keyword.\n"
        "Would you like to install it now?"
    )


class SnapMissingLinuxError(SnapcraftError):
    fmt = (
        "You need multipass installed to build snaps which use the base keyword.\n"
        "Enable snap support (https://docs.snapcraft.io/core/install) and run:\n"
        "snap install multipass --classic --beta"
    )