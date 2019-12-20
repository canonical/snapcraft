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

import enum
import logging
import os
import re
import shlex

from copy import deepcopy
from snapcraft import yaml_utils
from typing import Any, Dict, List, Optional, Sequence  # noqa: F401

from . import errors
from ._utils import _executable_is_valid
from .command import _massage_command
from .desktop import DesktopFile


logger = logging.getLogger(__name__)
_COMMAND_PATTERN = re.compile("^[A-Za-z0-9. _#:$-][A-Za-z0-9/. _#:$-]*$")
_MASSAGED_BASES = ["core", "core18"]
_FMT_SNAPD_WRAPPER = (
    "A shell wrapper will be generated for command {!r} as it does not conform "
    "with the command pattern expected by the runtime. "
    "Commands must be relative to the prime directory and can only consist "
    "of alphanumeric characters, spaces, and the following special characters: "
    "/ . _ # : $ -"
)


@enum.unique
class ApplicationAdapter(enum.Enum):
    NONE = 1
    LEGACY = 2
    FULL = 3


class Application:
    """Representation of an app entry in snapcraft.yaml"""

    def __init__(
        self,
        *,
        app_name: str,
        app_properties: Dict[str, Any] = None,
        adapter: ApplicationAdapter = ApplicationAdapter.LEGACY,
        command: Optional[str] = None,
        command_chain: Optional[List[str]] = None,
        desktop: str = None,
        passthrough: Dict[str, Any] = None,
        post_stop_command: Optional[str] = None,
        stop_command: Optional[str] = None,
    ) -> None:
        """Initialize an application entry.

        TODO: Eliminate use of app_properties.
        """
        self._app_name = app_name

        self._app_properties: Dict[str, Any] = dict()
        if app_properties:
            self._app_properties = app_properties

        self.adapter = adapter
        self.command = command

        self.command_chain: List[str] = list()
        if command_chain:
            self.command_chain = command_chain

        self.desktop = desktop

        self.passthrough: Dict[str, Any] = dict()
        if passthrough:
            self.passthrough = passthrough

        self.post_stop_command = post_stop_command
        self.stop_command = stop_command

    @property
    def app_name(self) -> str:
        """Read-only to ensure consistency with Snap dictionary mappings."""
        return self._app_name

    def can_massage_commands(self, *, base: Optional[str]) -> bool:
        return base in _MASSAGED_BASES

    def can_use_wrapper(self, base: Optional[str]) -> bool:
        """Return if an wrapper should be allowed for app entries."""
        # Force use of no wrappers when command-chain is set.
        if self.command_chain:
            return False

        # We only allow wrappers for core and core18.
        if not self.can_massage_commands(base=base):
            return False

        # Now that command-chain and bases have been checked for,
        # check if the none adapter has been forced.
        if self.adapter == ApplicationAdapter.NONE:
            return False

        return True

    def _prime_command(
        self, *, command: str, command_name: str, base: Optional[str], prime_dir: str
    ) -> str:
        """Finalize and prime command, massaging as necessary.

        If a wrapper is required, it will be generated and primed.  The
        command returned will point to the wrapper (the command will be
        preserved in the wrapper).

        Check if command is in prime_dir and raise exception if not valid."""

        if self.can_massage_commands(base=base):
            command = _massage_command(command=command, prime_dir=prime_dir)

        # Check to see if the command requires a wrapper.
        if command.startswith("/") or not _COMMAND_PATTERN.match(command):
            if not self.can_use_wrapper(base):
                raise errors.InvalidAppCommandFormatError(command, self._app_name)

            if not _COMMAND_PATTERN.match(command):
                logger.warning(_FMT_SNAPD_WRAPPER.format(command))

            # Write command wrapper and update command to point to it.
            command = self._write_command_wrapper(
                command=command, command_name=command_name, prime_dir=prime_dir
            )

        command_parts = shlex.split(command)
        command_path = os.path.join(prime_dir, command_parts[0])
        if not _executable_is_valid(command_path):
            raise errors.InvalidAppCommandNotExecutable(
                command=command, app_name=self._app_name
            )

        return command

    def prime_commands(self, *, base: Optional[str], prime_dir: str) -> None:
        if self.command:
            self.command = self._prime_command(
                command=self.command,
                command_name="command",
                base=base,
                prime_dir=prime_dir,
            )

        if self.post_stop_command:
            self.post_stop_command = self._prime_command(
                command=self.post_stop_command,
                command_name="post-stop-command",
                base=base,
                prime_dir=prime_dir,
            )

        if self.stop_command:
            self.stop_command = self._prime_command(
                command=self.stop_command,
                command_name="stop-command",
                base=base,
                prime_dir=prime_dir,
            )

    def write_application_desktop_file(
        self, snap_name: str, prime_dir: str, gui_dir: str, icon_path: Optional[str]
    ) -> None:
        if not self.desktop:
            return

        desktop_file = DesktopFile(
            snap_name=snap_name,
            app_name=self.app_name,
            filename=self.desktop,
            prime_dir=prime_dir,
        )

        desktop_file.write(gui_dir=gui_dir, icon_path=icon_path)

    def _write_command_wrapper(
        self, *, command_name: str, command: str, prime_dir: str
    ) -> str:
        """Write command wrapper for this command."""
        wrapped_command_name = f"{command_name}-{self._app_name}.wrapper"
        command_wrapper = os.path.join(prime_dir, wrapped_command_name)

        # We cannot exec relative paths in our wrappers.
        if not command.startswith("/"):
            command = os.path.join("$SNAP", command)

        with open(command_wrapper, "w+") as command_file:
            print("#!/bin/sh", file=command_file)
            print('exec {} "$@"'.format(command), file=command_file)

        os.chmod(command_wrapper, 0o755)
        return wrapped_command_name

    def validate_command_chain_executables(self, prime_dir: str) -> None:
        for item in self.command_chain:
            executable_path = os.path.join(prime_dir, item)

            # command-chain entries must always be relative to the root of
            # the snap, i.e. PATH is not used.
            if not _executable_is_valid(executable_path):
                raise errors.InvalidCommandChainError(item, self.app_name)

    def validate(self) -> None:
        """Validate application, raising exception on error."""
        if self.adapter == ApplicationAdapter.NONE and self.command_chain:
            raise errors.CommandChainWithIncompatibleAdapterError(
                app_name=self.app_name, adapter=self.adapter.name
            )

    @classmethod
    def from_dict(cls, *, app_dict: Dict[str, Any], app_name: str) -> "Application":
        """Create application from dictionary."""

        app_dict = deepcopy(app_dict)

        adapter_string = app_dict.get("adapter", "legacy").upper()
        adapter = ApplicationAdapter[adapter_string]

        return Application(
            app_name=app_name,
            app_properties=app_dict,
            adapter=adapter,
            command=app_dict.get("command", None),
            command_chain=app_dict.get("command-chain", None),
            desktop=app_dict.get("desktop", None),
            passthrough=app_dict.get("passthrough", None),
            post_stop_command=app_dict.get("post-stop-command", None),
            stop_command=app_dict.get("stop-command", None),
        )

    def to_dict(self) -> Dict[str, Any]:
        """Returns and ordered dictonary with the transformed app entry."""

        app_dict = deepcopy(self._app_properties)

        if self.command:
            app_dict["command"] = self.command

        if self.command_chain and self.adapter != ApplicationAdapter.NONE:
            app_dict["command-chain"] = self.command_chain

        if self.post_stop_command:
            app_dict["post-stop-command"] = self.post_stop_command

        # Adjust socket values to formats snap.yaml accepts
        sockets = app_dict.get("sockets", dict())
        for socket in sockets.values():
            mode = socket.get("socket-mode")
            if mode is not None:
                socket["socket-mode"] = yaml_utils.OctInt(mode)

        if self.stop_command:
            app_dict["stop-command"] = self.stop_command

        # Strip keys.
        for key in ["adapter", "desktop"]:
            if key in app_dict:
                app_dict.pop(key)

        # Apply passthrough keys.
        app_dict.update(self.passthrough)
        return app_dict

    def __repr__(self) -> str:
        return repr(self.__dict__)

    def __str__(self) -> str:
        return str(self.__dict__)
