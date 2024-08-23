# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
#  Copyright 2024 Canonical Ltd.
#
#  This program is free software: you can redistribute it and/or modify it
#  under the terms of the GNU Lesser General Public License version 3, as
#  published by the Free Software Foundation.
#
#  This program is distributed in the hope that it will be useful, but WITHOUT
#  ANY WARRANTY; without even the implied warranties of MERCHANTABILITY,
#  SATISFACTORY QUALITY, or FITNESS FOR A PARTICULAR PURPOSE.
#  See the GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""Models for assertion sets."""

import numbers
from typing import Any, Literal, Self

from craft_application import models


def cast_dict_scalars_to_strings(data: dict) -> dict[str, Any]:
    """Cast all scalars in a dictionary to strings.

    Supported scalar types are str, bool, and numbers.
    """
    return {_to_string(key): _to_string(value) for key, value in data.items()}


def _to_string(
        data: dict | list | str | int | float | str | bool | None
) -> dict[str, Any] | list | str | None:
    """Recurse through nested dicts and lists and cast scalar values to strings.

    Supported scalar types are str, bool, and numbers.
    """
    # start with a string as it is the most common scenario
    if isinstance(data, str):
        return data

    if isinstance(data, dict):
        return {_to_string(key): _to_string(value) for key, value in data.items()}

    if isinstance(data, list):
        return [_to_string(i) for i in data]

    if isinstance(data, (numbers.Number, bool)):
        return str(data)

    return data


class Assertion(models.CraftBaseModel):
    def marshal_scalars_as_strings(self) -> dict[str, Any]:
        """Marshal the object where all scalars are represented as strings."""
        data = self.marshal()
        return cast_dict_scalars_to_strings(data)


class Registry(models.CraftBaseModel):
    request: str
    storage: str | None = None
    access: str | None = None
    content: list[Self] | None = None


class Rules(models.CraftBaseModel):
    rules: list[Registry]


class EditableRegistryAssertion(Assertion):
    """Subset of a registry assertion that can be edited by the user."""
    account_id: str
    name: str
    summary: str | None = None
    revision: int | None = None
    views: dict[str, Rules]
    body: dict[str, Any] | str | None = None


class RegistryAssertion(EditableRegistryAssertion):

    type: Literal["registry"]

    authority_id: str
    """Same as the account id."""

    timestamp: str

    body_length: str | None = None
    """This is added by `snap sign -k <key-name>`"""

    sign_key_sha3_384: str | None = None
    """This is added by `snap sign -k <key-name>`"""


class RegistryList(models.CraftBaseModel):
    registry_list: list[RegistryAssertion] = []
