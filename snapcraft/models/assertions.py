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

"""Assertion models."""

import numbers
from collections import abc
from typing import Any, Literal

import pydantic
from craft_application import models
from typing_extensions import Self


def cast_dict_scalars_to_strings(data: dict) -> dict:
    """Cast all scalars in a dictionary to strings.

    Supported scalar types are str, bool, and numbers.
    """
    return {_to_string(key): _to_string(value) for key, value in data.items()}


def _to_string(data: Any) -> Any:
    """Recurse through nested dicts and lists and cast scalar values to strings.

    Supported scalar types are str, bool, and numbers.
    """
    # check for a string first, as it is the most common scenario
    if isinstance(data, str):
        return data

    if isinstance(data, abc.Mapping):
        return {_to_string(key): _to_string(value) for key, value in data.items()}

    if isinstance(data, abc.Collection):
        return [_to_string(i) for i in data]

    if isinstance(data, numbers.Number | bool):
        return str(data)

    return data


class ConfdbSchema(models.CraftBaseModel):
    """Access and data definitions for a specific facet of a snap or system."""

    request: str | None = None
    """Optional dot-separated path to access the field."""

    storage: str
    """Dot-separated storage path."""

    access: Literal["read", "write", "read-write"] | None = None
    """Access permissions for the field."""

    content: list[Self] | None = None
    """Optional nested rules."""


class Rules(models.CraftBaseModel):
    """A list of confdb schemas for a particular view."""

    summary: str | None = None
    """Optional summary for this view."""

    rules: list[ConfdbSchema]


class EditableConfdbSchemaAssertion(models.CraftBaseModel):
    """Subset of a confdb-schema assertion that can be edited by the user."""

    account_id: str
    """Issuer of the confdb-schema assertion and owner of the signing key."""

    name: str
    revision: int | None = 0

    summary: str | None = None
    """Optional summary for the entire confdb-schema."""

    views: dict[str, Rules]
    """A map of logical views of how the storage is accessed."""

    body: str | None = None
    """A JSON schema that defines the storage structure."""

    def marshal_scalars_as_strings(self) -> dict[str, Any]:
        """Marshal the model where all scalars are represented as strings."""
        return cast_dict_scalars_to_strings(self.marshal())


class ConfdbSchemaAssertion(EditableConfdbSchemaAssertion):
    """A full confdb-schema assertion containing editable and non-editable fields."""

    type: Literal["confdb-schema"]

    authority_id: str
    """Issuer of the confdb-schema assertion and owner of the signing key."""

    timestamp: str
    """Timestamp of when the assertion was issued."""

    body_length: str | None = None
    """Length of the body field."""

    sign_key_sha3_384: str | None = None
    """Signing key ID."""


# this will be a union for validation sets and confdb schemas once
# validation sets are migrated from the legacy codebase
Assertion = ConfdbSchemaAssertion

# this will be a union for editable validation sets and editable confdb schemas once
# validation sets are migrated from the legacy codebase
EditableAssertion = EditableConfdbSchemaAssertion
