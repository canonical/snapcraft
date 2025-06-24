# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2021-2023 Canonical Ltd.
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

from __future__ import annotations

import multiprocessing
import os
import pathlib
import re
import shutil
import sys
from collections.abc import Iterable
from getpass import getpass
from pathlib import Path

from craft_application.util import strtobool
from craft_cli import emit
from craft_parts import ProjectInfo
from craft_parts.sources.git_source import GitSource
from craft_platforms import DebianArchitecture

from snapcraft import errors


def get_supported_architectures() -> list[str]:
    """Get list of supported architectures for building."""
    return [arch.value for arch in DebianArchitecture]


def is_managed_mode() -> bool:
    """Check if snapcraft is running in a managed environment."""
    managed_flag = os.getenv("CRAFT_MANAGED_MODE", "n")
    return strtobool(managed_flag)


def get_managed_environment_home_path():
    """Path for home when running in managed environment."""
    return pathlib.Path("/root")


def get_managed_environment_project_path():
    """Path for project when running in managed environment."""
    return get_managed_environment_home_path() / "project"


def get_managed_environment_log_path():
    """Path for log when running in managed environment."""
    return pathlib.Path(
        "/tmp/snapcraft.log"  # noqa: S108 Probable insecure use of temp file
    )


def get_managed_environment_snap_channel() -> str | None:
    """User-specified channel to use when installing Snapcraft snap from Snap Store.

    :returns: Channel string if specified, else None.
    """
    return os.getenv("SNAPCRAFT_INSTALL_SNAP_CHANNEL")


def get_effective_base(
    *,
    base: str | None,
    build_base: str | None,
    project_type: str | None,
    name: str | None,
    translate_devel: bool = True,
) -> str | None:
    """Return the base to use to create the snap.

    Return the build-base if set.
    Exception:
    - "base" snaps will return name if build-base is not set.
    - For snaps with "devel" "build-base", if the "project_type" is "base",
      returns "devel". Otherwise:
        - if "translate_devel" is True, returns the "base";
        - otherwise, returns "devel".
    """
    if project_type == "base":
        return build_base if build_base else name

    if build_base is not None:
        if build_base == "devel" and translate_devel:
            return base
        return build_base

    return base


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

    max_count_env = os.environ.get("SNAPCRAFT_MAX_PARALLEL_BUILD_COUNT", "")
    try:
        max_count = int(max_count_env)
        if max_count > 0:
            build_count = min(build_count, max_count)
    except ValueError:
        emit.debug(f"Invalid SNAPCRAFT_MAX_PARALLEL_BUILD_COUNT {max_count_env!r}")

    return build_count


def confirm_with_user(prompt_text: str, default: bool = False) -> bool:
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

    with emit.pause():
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
    items: Iterable[str],
    conjunction: str,
    item_format: str = "{!r}",
    sort: bool = True,
) -> str:
    """Format a list into a human-readable string.

    :param items: list to humanize.
    :param conjunction: the conjunction used to join the final element to
                        the rest of the list (e.g. 'and').
    :param item_format: format string to use per item.
    :param sort: if true, sort the list.
    """
    if not items:
        return ""

    quoted_items = [item_format.format(item) for item in items]

    if sort:
        quoted_items = sorted(quoted_items)

    if len(quoted_items) == 1:
        return quoted_items[0]

    humanized = ", ".join(quoted_items[:-1])

    if len(quoted_items) > 2:
        humanized += ","

    return f"{humanized} {conjunction} {quoted_items[-1]}"


def get_common_ld_library_paths(prime_dir: Path, arch_triplet: str | None) -> list[str]:
    """Return common existing PATH entries for a snap.

    :param prime_dir: Path to the prime directory.
    :param arch_triplet: Architecture triplet of target arch. If None, the list of paths
    will not contain architecture-specific paths.

    :returns: List of common library paths in the prime directory that exist.
    """
    paths = [
        prime_dir / "lib",
        prime_dir / "usr" / "lib",
    ]

    if arch_triplet:
        paths.extend(
            [
                prime_dir / "lib" / arch_triplet,
                prime_dir / "usr" / "lib" / arch_triplet,
            ]
        )

    return [str(p) for p in paths if p.exists()]


def get_ld_library_paths(prime_dir: Path, arch_triplet: str | None) -> str:
    """Return a usable in-snap LD_LIBRARY_PATH variable.

    :param prime_dir: Path to the prime directory.
    :param arch_triplet: Architecture triplet of target arch. If None, LD_LIBRARY_PATH
    will not contain architecture-specific paths.

    :returns: The LD_LIBRARY_PATH environment variable to be used for the snap.
    """
    paths = ["${SNAP_LIBRARY_PATH}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"]
    # Add the default LD_LIBRARY_PATH
    paths += get_common_ld_library_paths(prime_dir, arch_triplet)

    ld_library_path = ":".join(paths)

    return re.sub(str(prime_dir), "$SNAP", ld_library_path)


def get_host_tool(command_name: str) -> str:
    """Return the full path of the given host tool.

    :param command_name: the name of the command to resolve a path for.
    :return: Path to command

    :raises SnapcraftError: if command_name was not found.
    """
    tool = shutil.which(command_name)
    if not tool:
        raise errors.SnapcraftError(
            f"A tool snapcraft depends on could not be found: {command_name!r}",
            resolution="Ensure the tool is installed and available, and try again.",
        )
    return tool


def get_snap_tool(command_name: str) -> str:
    """Return the path of a command found in the snap.

    If snapcraft is not running as a snap, shutil.which() is used
    to resolve the command using PATH.

    :param command_name: the name of the command to resolve a path for.
    :return: Path to command

    :raises SnapcraftError: if command_name was not found.
    """
    if os.environ.get("SNAP_NAME") != "snapcraft":
        return get_host_tool(command_name)

    snap_path = os.getenv("SNAP")
    if snap_path is None:
        raise RuntimeError(
            "The SNAP environment variable is not defined, but SNAP_NAME is?"
        )

    command_path = _find_command_path_in_root(snap_path, command_name)

    if command_path is None:
        raise errors.SnapcraftError(
            f"Cannot find snap tool {command_name!r}",
            resolution="Please report this error to the Snapcraft maintainers.",
        )

    return command_path


def _find_command_path_in_root(root: str, command_name: str) -> str | None:
    for bin_directory in (
        "usr/local/sbin",
        "usr/local/bin",
        "usr/sbin",
        "usr/bin",
        "sbin",
        "bin",
    ):
        path = os.path.join(root, bin_directory, command_name)
        if os.path.exists(path):
            return path

    return None


def process_version(version: str | None) -> str:
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


def is_snapcraft_running_from_snap() -> bool:
    """Check if snapcraft is running from the snap."""
    return os.getenv("SNAP_NAME") == "snapcraft" and os.getenv("SNAP") is not None


def get_prime_dirs_from_project(project_info: ProjectInfo) -> dict[str | None, Path]:
    """Get a mapping of component names to prime directories from a ProjectInfo.

    'None' maps to the default prime directory.

    :param project_info: The ProjectInfo to get the prime directory mapping from.
    """
    partition_prime_dirs = project_info.prime_dirs
    component_prime_dirs: dict[str | None, Path] = {None: project_info.prime_dir}

    # strip 'component/' prefix so that the component name is the key
    for partition, prime_dir in partition_prime_dirs.items():
        if partition and partition.startswith("component/"):
            component = partition.split("/", 1)[1]
            component_prime_dirs[component] = prime_dir

    return component_prime_dirs
