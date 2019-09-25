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

import os
from copy import deepcopy
from typing import Any, Dict, Optional, Sequence

from . import errors
from ._utils import _executable_is_valid
from .command import Command
from .desktop import DesktopFile
from snapcraft import yaml_utils


_COMMAND_ENTRIES = ["command", "stop-command"]
_MASSAGED_BASES = ["core", "core18"]


class Application:
    """Representation of an app entry in snapcraft.yaml"""

    def __init__(
        self,
        *,
        app_name: str,
        app_properties: Dict[str, Any],
        base: str,
        prime_dir: str
    ) -> None:
        """Initialize an application entry.

        :param str app_name: the name of the application.
        :param dict app_properties: the snapcraft.yaml application entry under
                                    apps.
        :param str prime_dir: directory containing the final assets to a snap.
        :param str base:
        """
        self._app_name = app_name
        self._app_properties = deepcopy(app_properties)
        self._base = base
        self._prime_dir = prime_dir

        # App Properties that should not make it to snap.yaml
        self._adapter = self._app_properties.pop("adapter", None)
        self._desktop_file = self._app_properties.pop("desktop", None)

        self._commands = self._get_commands()
        self._verify_paths()

    def _is_adapter(self, adapter: str) -> bool:
        if self._adapter is None:
            return False

        return self._adapter == adapter

    def _can_use_wrapper(self) -> bool:
        """Return if an wrapper should be allowed for app entries."""
        # Force use of no wrappers when command-chain is set.
        if "command-chain" in self._app_properties:
            return False

        # We only allow wrappers for core and core18.
        if self._base not in _MASSAGED_BASES:
            return False

        # Now that command-chain and bases have been checked for,
        # check if the none adapter has been forced.
        if self._is_adapter("none"):
            return False

        return True

    def _get_commands(self) -> Dict[str, Command]:
        can_use_wrapper = self._can_use_wrapper()
        commands = dict()
        for c in _COMMAND_ENTRIES:
            if c in self._app_properties:
                commands[c] = Command(
                    app_name=self._app_name,
                    command_name=c,
                    command=self._app_properties[c],
                    prime_dir=self._prime_dir,
                    can_use_wrapper=can_use_wrapper,
                    massage_command=self._base in _MASSAGED_BASES,
                )

        return commands

    def _verify_paths(self) -> None:
        for item in self._app_properties.get("command-chain", []):
            executable_path = os.path.join(self._prime_dir, item)

            # command-chain entries must always be relative to the root of
            # the snap, i.e. PATH is not used.
            if not _executable_is_valid(executable_path):
                raise errors.InvalidCommandChainError(item, self._app_name)

    def generate_desktop_file(
        self, *, snap_name: str, gui_dir: str, icon_path: Optional[str] = None
    ) -> None:
        if self._desktop_file is None:
            return

        desktop_file = DesktopFile(
            snap_name=snap_name,
            app_name=self._app_name,
            filename=self._desktop_file,
            prime_dir=self._prime_dir,
        )
        desktop_file.write(gui_dir=gui_dir, icon_path=icon_path)

    def generate_command_wrappers(self) -> None:
        for command in self._commands.values():
            command.generate_wrapper()

    def _fix_sockets(self) -> None:
        # Adjust socket values to formats snap.yaml accepts
        sockets = self._app_properties.get("sockets", dict())
        for socket in sockets.values():
            mode = socket.get("socket-mode")
            if mode is not None:
                socket["socket-mode"] = yaml_utils.OctInt(mode)

    def get_yaml(self, *, prepend_command_chain: Sequence[str]) -> Dict[str, Any]:
        """Returns and ordered dictonary with the transformed app entry."""
        for command_entry, command in self._commands.items():
            self._app_properties[command_entry] = command.get_command()

        if "command-chain" in self._app_properties or prepend_command_chain:
            self._app_properties[
                "command-chain"
            ] = prepend_command_chain + self._app_properties.get(
                "command-chain", list()
            )

        self._fix_sockets()

        return self._app_properties
