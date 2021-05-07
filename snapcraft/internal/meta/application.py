# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019-2021 Canonical Ltd
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
import os
from copy import deepcopy
from typing import Any, Dict, List, Optional, Sequence  # noqa: F401

from snapcraft import yaml_utils

from . import errors
from ._utils import _executable_is_valid
from .command import Command
from .desktop import DesktopFile

_COMMAND_ENTRIES = ["command", "stop-command"]
_MASSAGED_BASES = ["core", "core18"]


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
        adapter: ApplicationAdapter,
        desktop: str = None,
        install_mode: str = None,
        command_chain: List[str] = None,
        passthrough: Dict[str, Any] = None,
        commands: Dict[str, Command] = None
    ) -> None:
        """Initialize an application entry.

        TODO: Eliminate use of app_properties.
        """
        self._app_name = app_name

        self._app_properties: Dict[str, Any] = dict()
        if app_properties:
            self._app_properties = app_properties

        self.adapter = adapter
        self.desktop = desktop
        self.install_mode = install_mode

        self.command_chain: List[str] = list()
        if command_chain:
            self.command_chain = command_chain

        self.commands: Dict[str, Command] = dict()
        if commands:
            self.commands = commands

        self.passthrough: Dict[str, Any] = dict()
        if passthrough:
            self.passthrough = passthrough

    @property
    def app_name(self) -> str:
        """Read-only to ensure consistency with Snap dictionary mappings."""
        return self._app_name

    def can_use_wrapper(self, base: Optional[str]) -> bool:
        """Return if an wrapper should be allowed for app entries."""
        # Force use of no wrappers when command-chain is set.
        if self.command_chain:
            return False

        # We only allow wrappers for core and core18.
        if not self._massage_commands(base=base):
            return False

        # Now that command-chain and bases have been checked for,
        # check if the none adapter has been forced.
        if self.adapter == ApplicationAdapter.NONE:
            return False

        return True

    def _massage_commands(self, *, base: Optional[str]) -> bool:
        return base in _MASSAGED_BASES

    def prime_commands(self, *, base: Optional[str], prime_dir: str) -> None:
        can_use_wrapper = self.can_use_wrapper(base)
        massage_command = self._massage_commands(base=base)
        for command in self.commands.values():
            command.prime_command(
                can_use_wrapper=can_use_wrapper,
                massage_command=massage_command,
                prime_dir=prime_dir,
            )

    def write_command_wrappers(self, *, prime_dir: str) -> None:
        for command in self.commands.values():
            command.write_wrapper(prime_dir=prime_dir)

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

    def validate_command_chain_executables(self, prime_dir: str) -> None:
        for item in self.command_chain:
            executable_path = os.path.join(prime_dir, item)

            # command-chain entries must always be relative to the root of
            # the snap, i.e. PATH is not used.
            if not _executable_is_valid(executable_path):
                raise errors.InvalidCommandChainError(item, self.app_name)

    def validate(self) -> None:
        """Validate application, raising exception on error."""

        # No checks here (yet).
        return

    @classmethod
    def from_dict(cls, *, app_dict: Dict[str, Any], app_name: str) -> "Application":
        """Create application from dictionary."""

        app_dict = deepcopy(app_dict)

        adapter_string = app_dict.get("adapter", "full").upper()
        adapter = ApplicationAdapter[adapter_string]

        app = Application(
            app_name=app_name,
            app_properties=app_dict,
            adapter=adapter,
            desktop=app_dict.get("desktop", None),
            install_mode=app_dict.get("install-mode", None),
            command_chain=app_dict.get("command-chain", None),
            passthrough=app_dict.get("passthrough", None),
        )

        # Populate commands from app_properties.
        for command_name in _COMMAND_ENTRIES:
            if command_name not in app_dict:
                continue

            app.commands[command_name] = Command(
                app_name=app_name,
                command_name=command_name,
                command=app_dict[command_name],
            )

        return app

    def to_dict(self) -> Dict[str, Any]:
        """Returns and ordered dictionary with the transformed app entry."""

        app_dict = deepcopy(self._app_properties)

        for command_name, command in self.commands.items():
            app_dict[command_name] = command.command

        if self.command_chain:
            app_dict["command-chain"] = self.command_chain

        if self.install_mode:
            app_dict["install-mode"] = self.install_mode

        # Adjust socket values to formats snap.yaml accepts
        sockets = app_dict.get("sockets", dict())
        for socket in sockets.values():
            mode = socket.get("socket-mode")
            if mode is not None:
                socket["socket-mode"] = yaml_utils.OctInt(mode)

        # Strip keys.
        for key in ["adapter", "desktop"]:
            if key in app_dict:
                app_dict.pop(key)

        # Apply passthrough keys.
        app_dict.update(self.passthrough)

        # Ensure passthrough is removed.
        app_dict.pop("passthrough", None)

        return app_dict

    def __repr__(self) -> str:
        return repr(self.__dict__)

    def __str__(self) -> str:
        return str(self.__dict__)
