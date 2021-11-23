# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

from typing import Any, Dict

import jsonschema

from ._api_schema import WHOAMI_JSONSCHEMA

"""
This module holds representations for results for the v2 whoami
API endpoint provided by the Snap Store.

The full API is documented on
https://dashboard.snapcraft.io/docs/v2/en/tokens.html#api-tokens-whoami
"""


class Account:
    """Represent Account information for a whoami response."""

    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "Account":
        jsonschema.validate(payload, WHOAMI_JSONSCHEMA["properties"]["account"])
        return cls(
            email=payload["email"],
            account_id=payload["id"],
            name=payload["name"],
            username=payload["username"],
        )

    def marshal(self) -> Dict[str, Any]:
        return {
            "email": self.email,
            "id": self.account_id,
            "name": self.name,
            "username": self.username,
        }

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__}: {self.email!r}>"

    def __init__(
        self, *, email: str, account_id: str, name: str, username: str
    ) -> None:
        self.email = email
        self.account_id = account_id
        self.name = name
        self.username = username


class WhoAmI:
    """Represent the data returned from the whoami Snap Store endpoint."""

    @classmethod
    def unmarshal(cls, payload: Dict[str, Any]) -> "WhoAmI":
        jsonschema.validate(payload, WHOAMI_JSONSCHEMA)
        return cls(account=Account.unmarshal(payload["account"]))

    def marshal(self) -> Dict[str, Any]:
        return {
            "account": self.account.marshal(),
        }

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__}: {self.account.email!r}>"

    def __init__(self, *, account: Account) -> None:
        self.account = account
