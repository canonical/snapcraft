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

import re
from collections import OrderedDict
from typing import Any, Dict, List, Optional

from snapcraft.internal.meta.errors import HookValidationError


class Hook:
    """Representation of a generic snap hook."""

    def __init__(
        self,
        *,
        hook_name: str,
        command_chain: Optional[List[str]] = None,
        plugs: Optional[List[str]] = None,
        passthrough: Optional[Dict[str, Any]] = None,
    ) -> None:
        self._hook_name = hook_name

        self.command_chain: List[str] = list()
        if command_chain:
            self.command_chain = command_chain

        self.plugs: List[str] = list()
        if plugs:
            self.plugs = plugs

        self.passthrough: Dict[str, Any] = dict()
        if passthrough:
            self.passthrough = passthrough

    @property
    def hook_name(self) -> str:
        """Read-only to ensure consistency with Snap dictionary mappings."""

        return self._hook_name

    def _validate_name(self) -> None:
        """Validate hook name."""

        if not re.match("^[a-z](?:-?[a-z0-9])*$", self.hook_name):
            raise HookValidationError(
                hook_name=self.hook_name,
                message="{!r} is not a valid hook name. Hook names consist of lower-case alphanumeric characters and hyphens. They cannot start or end with a hyphen.".format(
                    self.hook_name
                ),
            )

    def _validate_command_chain(self) -> None:
        """Validate command-chain names."""

        # Would normally get caught/handled by schema validation.
        for command in self.command_chain:
            if not re.match("^[A-Za-z0-9/._#:$-]*$", command):
                raise HookValidationError(
                    hook_name=self.hook_name,
                    message=f"{command!r} is not a valid command-chain command.",
                )

    def validate(self) -> None:
        """Validate hook, raising exception if invalid."""

        self._validate_name()
        self._validate_command_chain()

    @classmethod
    def from_dict(cls, hook_dict: Dict[str, Any], hook_name: str) -> "Hook":
        """Create hook from dictionary."""

        return Hook(
            hook_name=hook_name,
            command_chain=hook_dict.get("command-chain", None),
            plugs=hook_dict.get("plugs", None),
            passthrough=hook_dict.get("passthrough", None),
        )

    def to_dict(self) -> Dict[str, Any]:
        """Create dictionary from hook."""

        hook_dict: Dict[str, Any] = OrderedDict()

        if self.command_chain:
            hook_dict["command-chain"] = self.command_chain

        if self.plugs:
            hook_dict["plugs"] = self.plugs

        # Apply passthrough keys.
        hook_dict.update(self.passthrough)
        return hook_dict

    def __repr__(self) -> str:
        return repr(self.__dict__)

    def __str__(self) -> str:
        return str(self.__dict__)
