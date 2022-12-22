# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2021 Canonical Ltd
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

from typing import Optional

from snapcraft_legacy.internal.errors import SnapcraftError, SnapcraftException

# dict of jsonschema validator -> cause pairs. Wish jsonschema just gave us
# better messages.


class MissingSnapcraftYamlError(SnapcraftError):

    fmt = (
        "Could not find {snapcraft_yaml_file_path}. Are you sure you are "
        "in the right directory?\n"
        "To start a new project, use `snapcraft init`"
    )

    def __init__(self, *, snapcraft_yaml_file_path):
        super().__init__(snapcraft_yaml_file_path=snapcraft_yaml_file_path)


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


class SnapcraftExperimentalExtensionsRequiredError(SnapcraftException):
    def __init__(self, *, extension_name: str) -> None:
        self.extension_name = extension_name

    def get_brief(self) -> str:
        return f"Experimental extension {self.extension_name!r} is required, but not enabled."

    def get_resolution(self) -> str:
        return "This extension may be enabled with the '--enable-experimental-extensions' parameter."


class UnsupportedBaseError(SnapcraftException):
    def __init__(self, *, base: str) -> None:
        self.base = base

    def get_brief(self) -> str:
        return f"Using {self.base!r} as a 'base' or 'build-base' is not supported."

    def get_details(self) -> Optional[str]:
        if self.base == "core":
            return (
                "'core' is currently under Extended Security Maintenance which "
                "requires a different version of Snapcraft to run."
            )
        return None

    def get_resolution(self) -> str:
        if self.base == "core":
            return (
                "Switch to Snapcraft's 4.x channel track or consider upgrading "
                "to a newer base."
            )
        else:
            return "Ensure a valid base is set in 'snapcraft.yaml'."

    def get_docs_url(self) -> Optional[str]:
        return "https://snapcraft.io/docs/base-snaps"
