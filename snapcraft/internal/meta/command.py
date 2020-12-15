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

import logging
import os
import pathlib
import re
import shlex
import shutil
from typing import Optional

from snapcraft.internal import common

from . import errors
from ._utils import _executable_is_valid

logger = logging.getLogger(__name__)
_COMMAND_PATTERN = re.compile("^[A-Za-z0-9. _#:$-][A-Za-z0-9/. _#:$-]*$")
_FMT_SNAPD_WRAPPER = (
    "A shell wrapper will be generated for command {!r} as it does not conform "
    "with the command pattern expected by the runtime. "
    "Commands must be relative to the prime directory and can only consist "
    "of alphanumeric characters, spaces, and the following special characters: "
    "/ . _ # : $ -"
)


def _get_shebang_from_file(file_path: pathlib.Path) -> Optional[str]:
    """Get the shebang from specified file."""
    with open(file_path, "rb") as exefile:
        if exefile.read(2) != b"#!":
            return None
        shebang_line = exefile.readline().strip().decode("utf-8")

    return shebang_line


class _SnapCommandResolver:
    def __init__(self, *, command: str, prime_path: pathlib.Path) -> None:
        self.original_command = command
        self._prime_path = prime_path
        self.interpreter: Optional[str] = None
        self.command = command

    def _find_executable(self, command: str) -> Optional[pathlib.Path]:
        """Find executable in common binary path locations.

        :return: Path to executable found, whether it is in searched
            prime directory paths, or on host using shutil.which().
        """
        binary_paths = (
            pathlib.Path(p, command)
            for p in common.get_bin_paths(root=self._prime_path)
        )

        # Final all potential hits, sorting to ensure consistent behavior.
        primed_binary_paths = sorted(
            [p for p in binary_paths if _executable_is_valid(p)]
        )

        # Warn if we have multiple potential hits.
        if len(primed_binary_paths) > 0:
            if len(primed_binary_paths) > 1:
                matches = [
                    str(p.relative_to(self._prime_path)) for p in primed_binary_paths
                ]
                logger.warning(
                    f"Multiple binaries matching for ambiguous command {command!r}: {matches!r}"
                )
            return primed_binary_paths[0]

        # Last chance to find in the prime_dir, mostly for backwards compatibility,
        # to find the executable, historical snaps like those built with the catkin
        # plugin will have roslaunch in a path like /opt/ros/bin/roslaunch.
        for root, _, files in os.walk(self._prime_path):
            file_path = pathlib.Path(root, command)
            if _executable_is_valid(str(file_path)):
                return file_path

        # Finally, check if it is part of the system.
        which_command = shutil.which(command)
        if which_command is not None:
            return pathlib.Path(which_command)

        return None

    def resolve_interpreter(self) -> Optional[str]:
        """Resolve the interpreter to a format suitable for use in a snap.yaml.

        Effectively performs the job of "/usr/bin/env", ignoring
        it if used as the (initial) interpreter.

        :return: Resolved and normalized interpreter path, if any.
            Incorporate arguments found in shebang.
        """
        interpreter = self.get_command_interpreter()
        if interpreter is None:
            return None

        # Remove the leading /usr/bin/env and resolve it now.
        interpreter_parts = shlex.split(interpreter, posix=False)
        if interpreter_parts[0] == "/usr/bin/env":
            interpreter_parts = interpreter_parts[1:]

        # If interpreter is absolute, ignore it.
        if pathlib.Path(interpreter_parts[0]).is_absolute():
            return None

        # Strip leading unnecessary $SNAP from command, if present.
        interpreter_parts[0] = re.sub(r"^\$SNAP/", "", interpreter_parts[0])

        # Check if relative to prime, where it should be found.
        if pathlib.Path(self._prime_path, interpreter_parts[0]).exists():
            return " ".join(interpreter_parts)

        # If not where it claims to be, search for backwards compatibility.
        interpreter_path = self._find_executable(command=interpreter_parts[0])

        if interpreter_path is None:
            # Interpreter was not found, note that this is not a hard error.
            logger.warning(
                f"The interpreter {interpreter_parts[0]!r} for {self.original_command!r} was not found."
            )
        else:
            # Interpreter was found in the prime directory, but required the
            # legacy search, or it was found on the host.  Log a warning.
            if self._prime_path in interpreter_path.parents:
                resolved_interpreter = interpreter_path.relative_to(
                    self._prime_path
                ).as_posix()
            else:
                resolved_interpreter = interpreter_path.as_posix()

            logger.warning(
                f"The interpreter {interpreter_parts[0]!r} for {self.original_command!r} was resolved to {resolved_interpreter!r}."
            )
            interpreter_parts[0] = resolved_interpreter

        return " ".join(interpreter_parts)

    def resolve_command(self) -> Optional[str]:
        """Resolve the given command, searching prime directory or host if needed.

        :return: Resolved and normalized command string.
        """
        command_parts = shlex.split(self.command, posix=False)

        # Strip leading unnecessary $SNAP from command, if present.
        command_parts[0] = re.sub(r"^\$SNAP/", "", command_parts[0])

        # Check if relative to prime, where it should be found.
        if os.path.exists(os.path.join(self._prime_path, command_parts[0])):
            return " ".join(command_parts)

        # If not where it claims to be, search for backwards compatibility.
        command_path = self._find_executable(command=command_parts[0])
        if command_path is None:
            return None

        # Command was found in the prime directory, but it required the
        # legacy search or was found on the host.  Log a warning.
        if self._prime_path in command_path.parents:
            resolved_command = command_path.relative_to(self._prime_path).as_posix()
        else:
            resolved_command = command_path.as_posix()

        logger.warning(
            f"The command {command_parts[0]!r} for {self.original_command!r} was resolved to {resolved_command!r}."
        )
        command_parts[0] = resolved_command

        return " ".join(command_parts)

    def get_command_interpreter(self) -> Optional[str]:
        """Get interpreter (shebang) for executable pointed to by self.command.

        :return: Interpreter string, else None if none found.
        """
        command_parts = shlex.split(self.command, posix=False)

        command_path = pathlib.Path(self._prime_path, command_parts[0])
        if not command_path.exists():
            return None

        return _get_shebang_from_file(command_path)

    def resolve(self) -> Optional[str]:
        """Resolve command to take into account interpreter and pathing,
        resolving ambiguous commands either relative to prime, or absolute
        to host/base.

        :return: Resolved and normalized command, accounting for the interpreter,
          if required.  None if command not found.
        """
        # Strip leading unneed $SNAP from command, if present.
        self.command = re.sub(r"^\$SNAP/", "", self.command)

        # Resolve interpreter, if any.
        self.interpreter = self.resolve_interpreter()

        # Resolve command.
        resolved_command = self.resolve_command()
        if resolved_command is None:
            return None

        self.command = resolved_command

        # Format interpreter and command for snap.yaml.
        if self.interpreter is not None:
            finalized_command = " ".join(
                [self.interpreter, pathlib.Path("$SNAP", self.command).as_posix()]
            )
        else:
            finalized_command = self.command

        # Issue final warnings for any changes.
        if finalized_command != self.original_command:
            if self.original_command.startswith("$SNAP/"):
                logger.warning(
                    f"Found unneeded '$SNAP/' in command {self.original_command!r}."
                )

            if self.interpreter is not None:
                logger.warning(
                    f"The command {self.original_command!r} has been changed to {finalized_command!r} to safely account for the interpreter."
                )
            else:
                logger.warning(
                    f"The command {self.original_command!r} has been changed to {finalized_command!r}."
                )

        return finalized_command

    @classmethod
    def resolve_snap_command_entry(
        cls, *, command: str, prime_path: pathlib.Path
    ) -> Optional[str]:
        """Resolve and normalize a given snap command entry, taking
        the interpreter into account, if appropriate.

        :param command: command to resolve and normalize.
        :param prime_path: path to prime directory.
        :return: Resolved and normalized command string suitable for a snap.yaml
            if no wrapper is required.
        """
        # If command is absolute, nothing to resolve.
        if command.startswith("/"):
            return command
        else:
            resolver = cls(command=command, prime_path=prime_path)
            return resolver.resolve()


class Command:
    """Representation of a command string."""

    def __str__(self) -> str:
        return self.command

    def __init__(self, *, app_name: str, command_name: str, command: str) -> None:
        self._app_name = app_name
        self._command_name = command_name
        self.command = command
        self.wrapped_command: Optional[str] = None
        self.massaged_command: Optional[str] = None

    @property
    def command_name(self) -> str:
        """Read-only to ensure consistency with app dictionary mappings."""
        return self._command_name

    @property
    def requires_wrapper(self) -> bool:
        if self.wrapped_command is not None:
            command = self.wrapped_command
        else:
            command = self.command

        return command.startswith("/") or not _COMMAND_PATTERN.match(command)

    @property
    def wrapped_command_name(self) -> str:
        """Return the relative in-snap path to the wrapper for command."""
        return "{command_name}-{app_name}.wrapper".format(
            command_name=self.command_name, app_name=self._app_name
        )

    def prime_command(
        self, *, can_use_wrapper: bool, massage_command: bool = True, prime_dir: str,
    ) -> str:
        """Finalize and prime command, massaging as necessary.

        Check if command is in prime_dir and raise exception if not valid."""

        if massage_command:
            resolved_command = _SnapCommandResolver.resolve_snap_command_entry(
                command=self.command, prime_path=pathlib.Path(prime_dir)
            )
            if resolved_command is None:
                raise errors.InvalidAppCommandNotFound(
                    command=self.command, app_name=self._app_name
                )
            self.command = resolved_command

        if self.requires_wrapper:
            if not can_use_wrapper:
                raise errors.InvalidAppCommandFormatError(self.command, self._app_name)
            self.wrapped_command = self.command
            if not _COMMAND_PATTERN.match(self.command):
                logger.warning(_FMT_SNAPD_WRAPPER.format(self.command))
            self.command = self.wrapped_command_name
        else:
            command_parts = shlex.split(self.command)
            command_path = os.path.join(prime_dir, command_parts[0])

            if not os.path.exists(command_path):
                raise errors.InvalidAppCommandNotFound(
                    command=self.command, app_name=self._app_name
                )

            if not _executable_is_valid(command_path):
                raise errors.InvalidAppCommandNotExecutable(
                    command=self.command, app_name=self._app_name
                )

        return self.command

    def write_wrapper(self, *, prime_dir: str) -> Optional[str]:
        """Write command wrapper if required for this command."""
        if self.wrapped_command is None:
            return None

        command_wrapper = os.path.join(prime_dir, self.wrapped_command_name)

        # We cannot exec relative paths in our wrappers.
        if self.wrapped_command.startswith("/"):
            command = self.wrapped_command
        else:
            command = os.path.join("$SNAP", self.wrapped_command)

        with open(command_wrapper, "w+") as command_file:
            print("#!/bin/sh", file=command_file)
            print('exec {} "$@"'.format(command), file=command_file)

        os.chmod(command_wrapper, 0o755)
        return command_wrapper
