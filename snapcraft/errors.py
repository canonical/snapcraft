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

import os
import subprocess

from craft_cli import CraftError

from . import const


class ClassicFallback(BaseException):
    """Temporary class to fall back to non craft-application launcher.

    Note that it inherits from BaseException so that it passes through
    craft-application, is caught by Snapcraft itself and then redirected to
    non-core24 codepaths.
    """


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


class ArchAllInvalid(SnapcraftError):
    """Architecture 'all' is invalid in this use case."""

    def __init__(self) -> None:
        super().__init__(
            "Cannot use architecture 'all'.",
            details="This command does not support using architecture 'all' in your snap.",
            resolution="Set your snap to architecture-dependent builds.",
            logpath_report=False,
            reportable=False,
            retcode=78,
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


class MissingBase(SnapcraftError):
    """No base defined in project."""

    def __init__(self) -> None:
        super().__init__(
            "Project file has no base or build-base.",
            docs_url="https://documentation.ubuntu.com/snapcraft/stable/how-to/crafting/specify-a-base",
        )


class MaintenanceBase(SnapcraftError):
    """Error for bases under ESM and no longer supported in this release."""

    def __init__(self, base: str | None) -> None:
        channel: str | None = None
        if base == "core":
            channel = "4.x"
        elif base == "core18":
            channel = "7.x"
        elif base == "core20":
            channel = "8.x"

        resolution: str | None = None
        if channel:
            resolution = f"Install from or refresh to the {channel!r} channel."

        super().__init__(
            f"{base!r} is not supported on this version of Snapcraft.",
            resolution=resolution,
            docs_url="https://documentation.ubuntu.com/snapcraft/stable/how-to/crafting/specify-a-base",
        )
        self.base = base


class StoreCredentialsUnauthorizedError(SnapcraftError):
    """Error raised for 401 responses from the Snap Store."""

    def __init__(self, message: str, *, resolution: str) -> None:
        super().__init__(
            message,
            resolution=resolution,
            docs_url="https://documentation.ubuntu.com/snapcraft/stable/how-to/publishing/authenticate",
        )


class SnapcraftAssertionError(SnapcraftError):
    """Error raised when an assertion (validation set or confdb schema) is invalid.

    Not to be confused with Python's built-in AssertionError.
    """


class SnapcraftAssertionWarning(SnapcraftAssertionError):
    """Error raised when there is a non-critical warning for the assertion."""


class SnapPackError(SnapcraftError):
    """Snapd packing error."""

    def _get_error_string_from_stderr(self, stderr: str | None) -> str | None:
        if stderr is None:
            return "snapd did not report an error"

        error_lines = (err for err in stderr.splitlines() if err.startswith("error: "))
        clean_error_lines = [err[len("error: ") :] for err in error_lines]
        # There shall only be one
        try:
            return clean_error_lines[-1]
        except IndexError:
            return "snapd did not report an error"

    def __init__(self, call_error: subprocess.CalledProcessError) -> None:
        super().__init__(
            message="Snapd failed to pack",
            details=self._get_error_string_from_stderr(call_error.stderr),
        )


class RemovedCommand(SnapcraftError):
    """Error for a command that was removed."""

    def __init__(self, removed_command: str, new_command: str) -> None:
        super().__init__(
            message=const.REMOVED_COMMAND_MESSAGE.format(
                old=removed_command, new=new_command
            ),
            resolution=const.REMOVED_COMMAND_RESOLUTION.format(new=new_command),
            retcode=os.EX_USAGE,
        )
