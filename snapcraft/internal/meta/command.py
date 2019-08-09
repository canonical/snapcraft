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
import os
import re
import shlex
import shutil
from typing import Optional

from . import errors
from ._utils import _executable_is_valid
from snapcraft.internal import common

_COMMAND_PATTERN = re.compile("^[A-Za-z0-9. _#:$-][A-Za-z0-9/. _#:$-]*$")


def _get_shebang_from_file(file_path: str) -> Optional[str]:
    """Returns the shebang from file_path."""
    if not os.path.exists(file_path):
        raise errors.ShebangNotFoundError()

    with open(file_path, "rb") as exefile:
        if exefile.read(2) != b"#!":
            raise errors.ShebangNotFoundError()
        shebang_line = exefile.readline().strip().decode("utf-8")

    if shebang_line.startswith("/usr/bin/env"):
        shebang = shebang_line.split()[1]
    else:
        shebang = shebang_line

    # if the shebang startswith / we should not extract it.
    if shebang.startswith("/"):
        raise errors.ShebangInRoot()

    return shebang


def _find_binary(*, binary: str, prime_dir: str) -> str:
    binary_paths = (
        os.path.join(p, binary) for p in common.get_bin_paths(root=prime_dir)
    )
    found_path = None
    for binary_path in binary_paths:
        if _executable_is_valid(binary_path):
            found_path = binary_path
            break
    else:
        found_path = shutil.which(binary)

    if found_path is None:
        raise errors.PrimedCommandNotFoundError(binary)

    return found_path


def _get_command_path(*, command: str, prime_dir: str) -> str:
    if command.startswith("/"):
        command_path = command
    # Remove a leading $SNAP if found.
    elif command.startswith("$SNAP/"):
        command_path = os.path.join(prime_dir, command[len("$SNAP/") :])
    else:
        command_path = os.path.join(prime_dir, command)

    return command_path


def _massage_command(*, command: str, prime_dir: str) -> str:
    # If command starts with "/" we have no option but to use a wrapper.
    if command.startswith("/"):
        return command

    command_parts = shlex.split(command)
    command_path = _get_command_path(command=command_parts[0], prime_dir=prime_dir)

    # Extract shebang
    with contextlib.suppress(errors.ShebangNotFoundError, errors.ShebangInRoot):
        shebang = _get_shebang_from_file(command_path)
        # Add the shebang (interpreter) to the front of the command.
        command_parts.insert(0, shebang)
        # And make sure the original command is prepended with $SNAP so it is
        # found.
        command_parts[1] = os.path.join("$SNAP", command_parts[1])
        command_path = _get_command_path(command=shebang, prime_dir=prime_dir)

    if not os.path.exists(command_path) and command_parts[0].startswith("$SNAP/"):
        raise errors.PrimedCommandNotFoundError(command_parts[0])
    elif not os.path.exists(command_path):
        command_path = _find_binary(binary=command_parts[0], prime_dir=prime_dir)

    if command_path.startswith(prime_dir):
        command_parts[0] = os.path.relpath(command_path, prime_dir)
    else:
        command_parts[0] = command_path

    return " ".join(command_parts)


class Command:
    """Representation of a command string."""

    def __str__(self) -> str:
        command = self.get_command()
        return command if command else ""

    def __init__(
        self,
        *,
        app_name: str,
        command_name: str,
        command: str,
        prime_dir: str,
        can_use_wrapper: bool
    ) -> None:
        self._app_name = app_name
        self._command_name = command_name
        self._original_command = command
        self._command = _massage_command(command=command, prime_dir=prime_dir)
        self._prime_dir = prime_dir
        self._generate_wrapper = None  # type: Optional[bool]
        self._can_use_wrapper = can_use_wrapper

    def _get_wrapped_command_name(self) -> str:
        """Return the relative in-snap path to the wrapper for command."""
        return "{command_name}-{app_name}.wrapper".format(
            command_name=self._command_name, app_name=self._app_name
        )

    def get_command(self) -> Optional[str]:
        # Verify that command matches a valid snapd pattern.
        # If the command starts with / we will need a wrapper, snapd strips the
        # leading /.
        if not self._command.startswith("/") and _COMMAND_PATTERN.match(self._command):
            command = self._command
            use_wrapper = False
        else:
            command = self._get_wrapped_command_name()
            use_wrapper = True

        if use_wrapper and not self._can_use_wrapper:
            raise errors.InvalidAppCommandFormatError(self._command, self._app_name)

        # Error for not being executable.
        command_parts = shlex.split(self._command)
        command_path = os.path.join(self._prime_dir, command_parts[0])
        if not use_wrapper and not _executable_is_valid(command_path):
            raise errors.InvalidAppCommandNotExecutable(
                command=self._command, app_name=self._app_name
            )

        self._generate_wrapper = use_wrapper
        return command

    def generate_wrapper(self) -> Optional[str]:
        if self._generate_wrapper is None:
            raise RuntimeError("get_command must be called first.")
        elif not self._generate_wrapper:
            return None

        command_wrapper = os.path.join(
            self._prime_dir, self._get_wrapped_command_name()
        )
        with open(command_wrapper, "w+") as command_file:
            print("#!/bin/sh", file=command_file)
            print('exec {} "$@"'.format(self._command), file=command_file)

        os.chmod(command_wrapper, 0o755)
        return command_wrapper
