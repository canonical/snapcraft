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
    """Error validatiing snapcraft.yaml."""


class ExtensionError(SnapcraftError):
    """Error during parts processing."""


class MetadataExtractionError(SnapcraftError):
    """Attempt to extract metadata from file was unsuccessful."""

    def __init__(self, filename: str, message: str) -> None:
        super().__init__(f"Error extracting metadata from {filename!r}: {message}")


class DesktopFileError(SnapcraftError):
    """Failed to create application desktop file."""

    def __init__(self, filename: str, message: str) -> None:
        super().__init__(f"Failed to generate desktop file {filename!r}: {message}")


class LegacyFallback(Exception):
    """Fall back to legacy snapcraft implementation."""
