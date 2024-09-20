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

from typing import Literal

import pydantic
from craft_application import models
from typing_extensions import Self


class Registry(models.CraftBaseModel):
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
    """A list of registries for a particular view."""

    rules: list[Registry]


class EditableRegistryAssertion(models.CraftBaseModel):
    """Subset of a registries assertion that can be edited by the user."""

    account_id: str
    """Issuer of the registry assertion and owner of the signing key."""

    name: str
    summary: str | None = None
    revision: int | None = 0

    views: dict[str, Rules]
    """A map of logical views of how the storage is accessed."""

    body: str | None = None
    """A JSON schema that defines the storage structure."""


class RegistryAssertion(EditableRegistryAssertion):
    """A full registries assertion containing editable and non-editable fields."""

    type: Literal["registry"]

    authority_id: str
    """Issuer of the registry assertion and owner of the signing key."""

    timestamp: str
    """Timestamp of when the assertion was issued."""

    body_length: str | None = None
    """Length of the body field."""

    sign_key_sha3_384: str | None = None
    """Signing key ID."""


class RegistriesList(models.CraftBaseModel):
    """A list of registry assertions."""

    registry_list: list[RegistryAssertion] = pydantic.Field(default_factory=list)


# this will be a union for validation sets and registries once
# validation sets are migrated from the legacy codebase
Assertion = RegistryAssertion

# this will be a union for editable validation sets and editable registries once
# validation sets are migrated from the legacy codebase
EditableAssertion = EditableRegistryAssertion
