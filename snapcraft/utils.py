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
import multiprocessing
import os
import pathlib
import platform
import re
import sys
from dataclasses import dataclass
from getpass import getpass
from pathlib import Path
from typing import Iterable, List, Optional

from craft_cli import emit
from craft_parts.sources.git_source import GitSource

from snapcraft import errors


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

_32BIT_USERSPACE_ARCHITECTURE = {
    "aarch64": "armv7l",
    "armv8l": "armv7l",
    "ppc64le": "ppc",
    "x86_64": "i686",
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
            emit.debug("Unable to locate 'os-release' file, using default values")
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
    os_platform_machine = get_os_platform().machine

    if platform.architecture()[0] == "32bit":
        userspace = _32BIT_USERSPACE_ARCHITECTURE.get(os_platform_machine)
        if userspace:
            os_platform_machine = userspace

    return ARCH_TRANSLATIONS.get(os_platform_machine, os_platform_machine)


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


def get_effective_base(
    *,
    base: Optional[str],
    build_base: Optional[str],
    project_type: Optional[str],
    name: Optional[str],
) -> Optional[str]:
    """Return the base to use to create the snap.

    Returns build-base if set, but if not, name is returned if the
    snap is of type base. For all other snaps, the base is returned
    as the build-base.
    """
    if build_base is not None:
        return build_base

    return name if project_type == "base" else base


def get_parallel_build_count() -> int:
    """Obtain the number of concurrent jobs to execute.

    Try different strategies to obtain the number of parallel jobs
    to execute. If they fail, assume the safe default of 1. The
    number of concurrent build jobs can be limited by setting the
    environment variable ``SNAPCRAFT_MAX_PARALLEL_BUILD_COUNT``.

    :return: The number of parallel jobs for the current host.
    """
    try:
        build_count = len(os.sched_getaffinity(0))
        emit.debug(f"CPU count (from process affinity): {build_count}")
    except AttributeError:
        # Fall back to multiprocessing.cpu_count()...
        try:
            build_count = multiprocessing.cpu_count()
            emit.debug(f"CPU count (from multiprocessing): {build_count}")
        except NotImplementedError:
            emit.progress(
                "Unable to determine CPU count; disabling parallel builds",
                permanent=True,
            )
            build_count = 1

    try:
        max_count = int(os.environ.get("SNAPCRAFT_MAX_PARALLEL_BUILD_COUNT", ""))
        if max_count > 0:
            build_count = min(build_count, max_count)
    except ValueError:
        emit.debug("Invalid SNAPCRAFT_MAX_PARALLEL_BUILD_COUNT value")

    return build_count


def confirm_with_user(prompt_text, default=False) -> bool:
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

    reply = str(input(prompt_text + choices)).lower().strip()
    if reply and reply[0] == "y":
        return True

    if reply and reply[0] == "n":
        return False

    return default


def prompt(prompt_text: str, *, hide: bool = False) -> str:
    """Prompt and return the entered string.

    :param prompt_text: string used for the prompt.
    :param hide: hide user input if True.
    """
    if is_managed_mode():
        raise RuntimeError("prompting not yet supported in managed-mode")

    if not sys.stdin.isatty():
        raise errors.SnapcraftError("prompting not possible with no tty")

    if hide:
        method = getpass
    else:
        method = input  # type: ignore

    with emit.pause():
        return str(method(prompt_text))


def humanize_list(
    items: Iterable[str], conjunction: str, item_format: str = "{!r}"
) -> str:
    """Format a list into a human-readable string.

    :param items: list to humanize.
    :param conjunction: the conjunction used to join the final element to
                        the rest of the list (e.g. 'and').
    :param item_format: format string to use per item.
    """
    if not items:
        return ""

    quoted_items = [item_format.format(item) for item in sorted(items)]
    if len(quoted_items) == 1:
        return quoted_items[0]

    humanized = ", ".join(quoted_items[:-1])

    if len(quoted_items) > 2:
        humanized += ","

    return f"{humanized} {conjunction} {quoted_items[-1]}"


def _get_common_ld_library_paths(prime_dir: Path, arch_triplet: str) -> List[str]:
    """Return common existing PATH entries for a snap."""
    paths = [
        prime_dir / "lib",
        prime_dir / "usr" / "lib",
        prime_dir / "lib" / arch_triplet,
        prime_dir / "usr" / "lib" / arch_triplet,
    ]

    return [str(p) for p in paths if p.exists()]


def get_ld_library_paths(prime_dir: Path, arch_triplet: str) -> str:
    """Return a usable in-snap LD_LIBRARY_PATH variable."""
    paths = ["${SNAP_LIBRARY_PATH}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"]
    # Add the default LD_LIBRARY_PATH
    paths += _get_common_ld_library_paths(prime_dir, arch_triplet)

    ld_library_path = ":".join(paths)

    return re.sub(str(prime_dir), "$SNAP", ld_library_path)


def process_version(version: Optional[str]) -> str:
    """Handle special version strings."""
    if version is None:
        raise ValueError("version cannot be None")

    new_version = version
    if version == "git":
        emit.progress("Determining the version from the project repo (version: git).")
        new_version = GitSource.generate_version()

    if new_version != version:
        emit.progress(f"Version has been set to {new_version!r}", permanent=True)

    return new_version
