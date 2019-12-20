# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

import contextlib
import logging
import os
import re
import shlex
import shutil
from typing import List, Optional

from . import errors
from ._utils import _executable_is_valid
from snapcraft.internal import common


logger = logging.getLogger(__name__)
_FMT_COMMAND_SNAP_STRIP = "Stripped '$SNAP/' from command {!r}."
_FMT_COMMAND_ROOT = (
    "The command {!r} was not found in the prime directory, it has been "
    "changed to {!r}."
)


def _get_shebang_from_file(file_path: str) -> List[str]:
    """Returns the shebang from file_path."""
    if not os.path.exists(file_path):
        raise errors.ShebangNotFoundError()

    with open(file_path, "rb") as exefile:
        if exefile.read(2) != b"#!":
            raise errors.ShebangNotFoundError()
        shebang_line = exefile.readline().strip().decode("utf-8")

    # posix is set to False to respect the quoting of variables.
    shebang_parts = shlex.split(shebang_line, posix=False)
    # remove the leading /usr/bin/env.
    if shebang_parts[0] == "/usr/bin/env":
        shebang_parts = shebang_parts[1:]
    # or if the shebang startswith /.
    elif shebang_parts[0].startswith("/"):
        raise errors.ShebangInRoot()

    return shebang_parts


def _find_executable(*, binary: str, prime_dir: str) -> str:
    found_path: Optional[str] = None
    binary_paths = (
        os.path.join(p, binary) for p in common.get_bin_paths(root=prime_dir)
    )
    for binary_path in binary_paths:
        if _executable_is_valid(binary_path):
            found_path = binary_path
            break
    else:
        # Last chance to find in the prime_dir, mostly for backwards compatibility,
        # to find the executable, historical snaps like those built with the catkin
        # plugin will have roslaunch in a path like /opt/ros/bin/roslaunch.
        for root, _, files in os.walk(prime_dir):
            if _executable_is_valid(os.path.join(root, binary)):
                found_path = os.path.join(root, binary)
                break
        else:
            # Finally, check if it is part of the system.
            found_path = shutil.which(binary)

    if found_path is None:
        raise errors.PrimedCommandNotFoundError(binary)

    return found_path


def _get_command_path(*, command: str, prime_dir: str) -> str:
    # Strip leading "/"
    command = re.sub(r"^/", "", command)
    # Strip leading "$SNAP/"
    command = re.sub(r"^\$SNAP/", "", command)

    return os.path.join(prime_dir, command)


def _massage_command(*, command: str, prime_dir: str) -> str:
    """Rewrite command to take into account interpreter and pathing.

    (1) Interpreter: if shebang is found in file, explicitly prepend
        the interpreter to the command string.  Fixup path of
        command with $SNAP if required so the interpreter is able
        to find the correct target.
    (2) Explicit path: attempt to find the executable if path
        is ambiguous.  If found in prime_dir, set the path relative
        to snap.

    Returns massaged command."""

    # If command starts with "/" we have no option but to use a wrapper.
    if command.startswith("/"):
        return command

    # command_parts holds the lexical split of a command entry.
    # posix is set to False to respect the quoting of variables.
    command_parts = shlex.split(command, posix=False)
    # Make a note now that $SNAP if found this path will not make it into the
    # resulting command entry.
    if command_parts[0].startswith("$SNAP/"):
        logger.warning(_FMT_COMMAND_SNAP_STRIP.format(command))
    # command_path is the absolute path to the real command (command_parts[0]),
    # used to verify that the executable is valid and potentially extract a
    # shebang.
    command_path = _get_command_path(command=command_parts[0], prime_dir=prime_dir)

    # Extract shebang, if the shebang is not root bound it will be inserted at the
    # beginning of command_parts. If the shebang is not found we will ignore it
    # for backwards compatibility (this might be part of a content interfaced
    # snap).
    # If a shebang is found, command_path is rewritten to point to the shebang.
    with contextlib.suppress(errors.ShebangNotFoundError, errors.ShebangInRoot):
        shebang_parts = _get_shebang_from_file(command_path)
        # Add the shebang (interpreter) to the front of the command.
        command_parts = shebang_parts + command_parts
        # And make sure the original command is prepended with $SNAP so it is
        # found and not already set.
        if not command_parts[1].startswith("$SNAP"):
            command_parts[1] = os.path.join("$SNAP", command_parts[1])
        command_path = _get_command_path(command=shebang_parts[0], prime_dir=prime_dir)

    # If the command is part of the snap (starts with $SNAP) it NEEDS to exist
    # within the prime directory.
    if not os.path.exists(command_path) and command_parts[0].startswith("$SNAP/"):
        raise errors.PrimedCommandNotFoundError(command_parts[0])
    # if the command is "pathless", make an attempt to find the executable within
    # the prime directory and as a last resort (for backwards compatibility),
    # at the root of the filesystem.
    elif not os.path.exists(command_path):
        command_path = _find_executable(binary=command_parts[0], prime_dir=prime_dir)

    # A command found within the prime directory will have a command_path that
    # starts with the prime directory leading the path. Make it relative to
    # this prime directory before replacing the original command.
    if command_path.startswith(prime_dir):
        command_parts[0] = os.path.relpath(command_path, prime_dir)
    # If not in the prime directory, add as is as it is pointing to the root
    # (most likely part of the base for this snap).
    else:
        logger.warning(_FMT_COMMAND_ROOT.format(command, command_path))
        command_parts[0] = command_path

    return " ".join(command_parts)
