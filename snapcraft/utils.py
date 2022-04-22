# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2021-2022 Canonical Ltd.
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

"""Utilities for snapcraft."""

import os
import pathlib
import platform
import sys
from dataclasses import dataclass
from typing import Optional

from craft_cli import emit


@dataclass
class OSPlatform:
    """Platform definition for a given host."""

    system: str
    release: str
    machine: str

    def __str__(self) -> str:
        """Return the string representation of an OSPlatform."""
        return f"{self.system}/{self.release} ({self.machine})"


# translations from what the platform module informs to the term deb and
# snaps actually use
ARCH_TRANSLATIONS = {
    "aarch64": "arm64",
    "armv7l": "armhf",
    "i686": "i386",
    "ppc": "powerpc",
    "ppc64le": "ppc64el",
    "x86_64": "amd64",
    "AMD64": "amd64",  # Windows support
}


def get_os_platform(filepath=pathlib.Path("/etc/os-release")):
    """Determine a system/release combo for an OS using /etc/os-release if available."""
    system = platform.system()
    release = platform.release()
    machine = platform.machine()

    if system == "Linux":
        try:
            with filepath.open("rt", encoding="utf-8") as release_file:
                lines = release_file.readlines()
        except FileNotFoundError:
            emit.trace("Unable to locate 'os-release' file, using default values")
        else:
            os_release = {}
            for line in lines:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                key, value = line.rstrip().split("=", 1)
                if value[0] == value[-1] and value[0] in ('"', "'"):
                    value = value[1:-1]
                os_release[key] = value
            system = os_release.get("ID", system)
            release = os_release.get("VERSION_ID", release)

    return OSPlatform(system=system, release=release, machine=machine)


def get_host_architecture():
    """Get host architecture in deb format suitable for base definition."""
    os_platform = get_os_platform()
    return ARCH_TRANSLATIONS.get(os_platform.machine, os_platform.machine)


def strtobool(value: str) -> bool:
    """Convert a string representation of truth to true (1) or false (0).

    :param value: a True value of 'y', 'yes', 't', 'true', 'on', and '1'
        or a False value of 'n', 'no', 'f', 'false', 'off', and '0'.
    :raises ValueError: if `value` is not a valid boolean value.
    """
    parsed_value = value.lower()

    if parsed_value in ("y", "yes", "t", "true", "on", "1"):
        return True
    if parsed_value in ("n", "no", "f", "false", "off", "0"):
        return False

    raise ValueError(f"Invalid boolean value of {value!r}")


def is_managed_mode() -> bool:
    """Check if snapcraft is running in a managed environment."""
    managed_flag = os.getenv("SNAPCRAFT_MANAGED_MODE", "n")
    return strtobool(managed_flag)


def get_managed_environment_home_path():
    """Path for home when running in managed environment."""
    return pathlib.Path("/root")


def get_managed_environment_project_path():
    """Path for project when running in managed environment."""
    return get_managed_environment_home_path() / "project"


def get_managed_environment_log_path():
    """Path for log when running in managed environment."""
    return pathlib.Path("/tmp/snapcraft.log")


def get_managed_environment_snap_channel() -> Optional[str]:
    """User-specified channel to use when installing Snapcraft snap from Snap Store.

    :returns: Channel string if specified, else None.
    """
    return os.getenv("SNAPCRAFT_INSTALL_SNAP_CHANNEL")


def confirm_with_user(prompt, default=False) -> bool:
    """Query user for yes/no answer.

    If stdin is not a tty, the default value is returned.

    If user returns an empty answer, the default value is returned.
    returns default value.

    :returns: True if answer starts with [yY], False if answer starts with [nN],
        otherwise the default.
    """
    if is_managed_mode():
        raise RuntimeError("confirmation not yet supported in managed-mode")

    if not sys.stdin.isatty():
        return default

    choices = " [Y/n]: " if default else " [y/N]: "

    reply = str(input(prompt + choices)).lower().strip()
    if reply and reply[0] == "y":
        return True

    if reply and reply[0] == "n":
        return False

    return default
