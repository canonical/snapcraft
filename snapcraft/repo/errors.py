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

"""Package repository error definitions."""

from typing import Optional

from snapcraft.errors import SnapcraftError


class PackageRepositoryError(SnapcraftError):
    """Package repository error base."""


class PackageRepositoryValidationError(PackageRepositoryError):
    """Package repository is invalid."""

    def __init__(
        self,
        url: str,
        brief: str,
        details: Optional[str] = None,
        resolution: Optional[str] = None,
    ):
        super().__init__(
            f"Invalid package repository for {url!r}: {brief}",
            details=details,
            resolution=resolution,
        )


class AptPPAInstallError(PackageRepositoryError):
    """Installation of a PPA repository failed."""

    def __init__(self, ppa: str, reason: str):
        super().__init__(
            f"Failed to install PPA {ppa!r}: {reason}",
            resolution="Verify PPA is correct and try again",
        )


class AptGPGKeyInstallError(PackageRepositoryError):
    """Installation of GPG key failed."""

    def __init__(
        self,
        output: str,
        *,
        key: Optional[str] = None,
        key_id: Optional[str] = None,
        key_server: Optional[str] = None,
    ):
        """Convert apt-key's output into a more user-friendly message."""
        message = output.replace(
            "Warning: apt-key output should not be parsed (stdout is not a terminal)",
            "",
        ).strip()

        # Improve error messages that we can.
        if (
            "gpg: keyserver receive failed: No data" in message
            and key_id
            and key_server
        ):
            message = f"GPG key {key_id!r} not found on key server {key_server!r}"
        elif (
            "gpg: keyserver receive failed: Server indicated a failure" in message
            and key_server
        ):
            message = f"unable to establish connection to key server {key_server!r}"
        elif (
            "gpg: keyserver receive failed: Connection timed out" in message
            and key_server
        ):
            message = (
                f"unable to establish connection to key server {key_server!r} "
                f"(connection timed out)"
            )

        details = ""
        if key:
            details += f"GPG key:\n{key}\n"
        if key_id:
            details += f"GPG key ID: {key_id}\n"
        if key_server:
            details += f"GPG key server: {key_server}"

        super().__init__(
            f"Failed to install GPG key: {message}",
            details=details,
            resolution="Verify any configured GPG keys",
        )
