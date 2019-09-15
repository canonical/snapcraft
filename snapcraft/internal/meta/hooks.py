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

from copy import deepcopy
from snapcraft.internal.meta.errors import HookValidationError
from typing import Any, Dict


class Hook:
    """Representation of a generic snap hook."""

    def __init__(self, *, hook_name: str) -> None:
        self._hook_name = hook_name
        self._hook_properties: Dict[str, Any] = dict()
        self.passthrough: Dict[str, Any] = dict()

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

    def validate(self) -> None:
        """Validate hook, raising exception if invalid."""

        self._validate_name()

    @classmethod
    def from_dict(cls, hook_dict: Dict[str, Any], hook_name: str) -> "Hook":
        """Create hook from dictionary."""

        hook = Hook(hook_name=hook_name)
        hook._hook_properties = deepcopy(hook_dict)

        if "passthrough" in hook._hook_properties:
            hook.passthrough = hook._hook_properties.pop("passthrough")

        return hook

    def to_dict(self) -> Dict[str, Any]:
        """Create dictionary from hook."""

        hook_dict = deepcopy(self._hook_properties)

        # Apply passthrough keys.
        hook_dict.update(self.passthrough)
        return hook_dict

    def __repr__(self) -> str:
        return repr(self.__dict__)

    def __str__(self) -> str:
        return str(self.__dict__)
