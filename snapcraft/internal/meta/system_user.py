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
from collections import OrderedDict
from typing import Any, Dict

from snapcraft.internal.meta import errors

logger = logging.getLogger(__name__)


@enum.unique
class SystemUserScope(enum.Enum):
    SHARED = 1


class SystemUser:
    def __init__(self, *, name: str, scope: SystemUserScope) -> None:
        self.name = name
        self.scope = scope

    @classmethod
    def from_dict(cls, *, user_name: str, user_dict: Dict[str, str]) -> "SystemUser":
        raw_scope = user_dict.get("scope", None)
        if raw_scope is None:
            raise errors.SystemUsernamesValidationError(
                name=user_name, message="'scope' is undefined"
            )

        try:
            scope = SystemUserScope[raw_scope.upper()]
        except KeyError:
            raise errors.SystemUsernamesValidationError(
                name=user_name, message=f"scope {raw_scope!r} is invalid"
            )

        return SystemUser(name=user_name, scope=scope)

    @classmethod
    def from_object(cls, *, user_object: Any, user_name: str) -> "SystemUser":
        if user_object is None:
            raise errors.SystemUsernamesValidationError(
                name=user_name, message="undefined user"
            )
        elif isinstance(user_object, str):
            try:
                scope = SystemUserScope[user_object.upper()]
            except KeyError:
                raise errors.SystemUsernamesValidationError(
                    name=user_name, message=f"scope {user_object!r} is invalid"
                )

            return SystemUser(name=user_name, scope=scope)
        elif isinstance(user_object, dict):
            return cls.from_dict(user_dict=user_object, user_name=user_name)

        raise errors.SystemUsernamesValidationError(
            name=user_name, message=f"unknown syntax for {user_object!r}"
        )

    def to_dict(self):
        user_dict = OrderedDict()
        user_dict["scope"] = self.scope.name.lower()
        return user_dict

    def validate(self):
        if not isinstance(self.scope, SystemUserScope):
            raise errors.SystemUsernamesValidationError(
                name=self.name, message=f"scope {self.scope!r} is invalid"
            )
