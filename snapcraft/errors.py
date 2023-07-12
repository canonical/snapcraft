# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

"""Snapcraft error definitions."""

from typing import Optional

from craft_cli import CraftError


class SnapcraftError(CraftError):
    """Failure in a Snapcraft operation."""


class FeatureNotImplemented(SnapcraftError):
    """Attempt to use an unimplemented feature."""

    def __init__(self, msg: str) -> None:
        super().__init__(f"Command or feature not implemented: {msg}")


class PartsLifecycleError(SnapcraftError):
    """Error during parts processing."""


class ProjectValidationError(SnapcraftError):
    """Error validating snapcraft.yaml."""


class ExtensionError(SnapcraftError):
    """Error during parts processing."""


class LegacyCredentialsParseError(SnapcraftError):
    """Error produced when parsing the legacy Store credentials."""


class MetadataExtractionError(SnapcraftError):
    """Attempt to extract metadata from file was unsuccessful."""

    def __init__(self, filename: str, message: str) -> None:
        super().__init__(f"Error extracting metadata from {filename!r}: {message}")


class DesktopFileError(SnapcraftError):
    """Failed to create application desktop file."""

    def __init__(self, filename: str, message: str) -> None:
        super().__init__(f"Failed to generate desktop file {filename!r}: {message}")


class FilePermissionError(SnapcraftError):
    """Insufficient permissions to access a file."""

    def __init__(self, filename: str, *, reason: str) -> None:
        super().__init__(
            f"{reason} in file {filename}",
            resolution=(
                "Make sure the file is part of the current project and its permissions "
                "and ownership are correct."
            ),
        )


class InvalidArchitecture(SnapcraftError):
    """The machine architecture is not supported.

    :param arch_name: The unsupported architecture name.
    """

    def __init__(self, arch_name: str):
        self.arch_name = arch_name
        super().__init__(
            f"Architecture {arch_name!r} is not supported.",
            resolution="Make sure the architecture name is correct.",
        )


class LinterError(SnapcraftError):
    """Snap linting returned an error.

    :param message: The error description.
    :exit_code: The shell return code to use.
    """

    def __init__(self, message: str, *, exit_code: int):
        self.exit_code = exit_code
        super().__init__(
            message,
            resolution="Make sure the issues are addressed or ignore the issues in snapcraft.yaml.",
        )


class ProjectMissing(SnapcraftError):
    """No snapcraft.yaml project found."""

    def __init__(self) -> None:
        super().__init__(
            "Could not find snap/snapcraft.yaml. Are you sure you are in the "
            "right directory?",
            resolution="To start a new project, use `snapcraft init`",
        )


class LegacyFallback(Exception):
    """Fall back to legacy snapcraft implementation."""


class MaintenanceBase(SnapcraftError):
    """Error for bases under ESM and no longer supported in this release."""

    def __init__(self, base) -> None:
        channel: Optional[str] = None
        if base == "core":
            channel = "4.x"
        elif base == "core18":
            channel = "7.x"

        resolution: Optional[str] = None
        if channel:
            resolution = f"Install from or refresh to the {channel!r} channel."

        super().__init__(
            f"{base!r} is not supported on this version of Snapcraft.",
            resolution=resolution,
            docs_url="https://snapcraft.io/docs/base-snaps",
        )


class StoreCredentialsUnauthorizedError(SnapcraftError):
    """Error raised for 401 responses from the Snap Store."""

    def __init__(self, message: str, *, resolution: str) -> None:
        super().__init__(
            message,
            resolution=resolution,
            docs_url="https://snapcraft.io/docs/snapcraft-authentication",
        )
