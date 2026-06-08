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
from typing import Annotated, Any, Literal

import pydantic
from craft_application import models
from typing_extensions import Self, override

SnapName = Annotated[str, pydantic.StringConstraints(max_length=40)]
SnapId = Annotated[str, pydantic.StringConstraints(max_length=40)]
Presence = Literal["required", "optional", "invalid"]


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


class Component(models.CraftBaseModel):
    """A Component in a Validation Set."""

    presence: Presence
    """Component presence"""

    revision: int | None = None
    """Component revision"""


class Snap(models.CraftBaseModel):
    """A Snap in a Validation Set."""

    name: SnapName
    """Snap name"""

    id: SnapId | None = None
    """Snap ID"""

    presence: Presence | None = None
    """Snap presence"""

    revision: int | None = None
    """Snap revision"""

    components: dict[str, Presence | Component] | None = None
    """Snap components"""


class EditableValidationSetAssertion(models.CraftBaseModel):
    """Subset of a validation-set that can be edited by the user.

    https://dashboard.snapcraft.io/docs/reference/v2/en/validation-sets.html#request-json-schema
    """

    account_id: str
    """The "account-id" assertion header"""

    name: str
    """The "name" assertion header"""

    revision: str | None = None
    """The "revision" assertion header"""

    sequence: int
    """The "sequence" assertion header"""

    snaps: Annotated[list[Snap], pydantic.Field(min_length=1)]
    """List of snaps in a Validation Set assertion"""

    def marshal_scalars_as_strings(self) -> dict[str, Any]:
        """Marshal the model where all scalars are represented as strings."""
        return cast_dict_scalars_to_strings(self.marshal())


class ValidationSetAssertion(EditableValidationSetAssertion):
    """A full validation set containing editable and non-editable fields.

    https://dashboard.snapcraft.io/docs/reference/v2/en/validation-sets.html#response-json-schema
    """

    authority_id: str
    """The "authority-id" assertion header"""

    series: str
    """The "series" assertion header"""

    sign_key_sha3_384: None = None
    """Signing key ID."""

    timestamp: str
    """The "timestamp" assertion header"""

    type: Literal["validation-set"]
    """The "type" assertion header"""

    @pydantic.model_validator(mode="before")
    @classmethod
    def remove_sign_key(cls, values: dict[str, Any]) -> dict[str, Any]:
        """Accept but always ignore the sign key.

        The store API can accept and return a signing key, but the original
        implementation in Snapcraft ignored it, so it's ignored here for compatibility.
        """
        values.pop("sign-key-sha3-384", None)
        return values


class ValidationSetHeaders(models.CraftBaseModel):
    """Assertion headers for a validation set."""

    headers: ValidationSetAssertion
    """Assertion headers"""


class ValidationAssertion(models.CraftBaseModel):
    """A validation assertion used to gate a snap at a specific revision.

    https://documentation.ubuntu.com/core/reference/assertions/validation
    """

    assertion_type: Literal["validation"] = pydantic.Field(alias="type")

    authority_id: str
    """Issuer of the validation."""

    revision: int | None = None
    """The revision of the validation itself."""

    series: str
    """Series for which this validation applies (typically "16")."""

    snap_id: str
    """ID of the snap that constrains (gates) the approved snap."""

    approved_snap_id: str
    """ID of the snap whose updates are gated."""

    approved_snap_revision: str
    """Revision of the snap whose updates are gated."""

    # Returned by the store but not in the docs
    approved_snap_name: str | None = None
    """Name of the snap whose updates are gated."""

    # Returned by the store but not in the docs
    sign_key_sha3_384: str | None = None
    """Signing key ID."""

    timestamp: str
    """When the validation was issued."""

    # Returned by the store but not in the docs
    required: bool | None = None
    """Whether this validation is required."""

    revoked: bool
    """Whether this validation has been revoked."""

    @pydantic.field_serializer("revoked")
    def _serialize_revoked(self, value: bool) -> str:
        """The revoked field must be 'true' or 'false' (lowercase) for `snap sign`."""
        return "true" if value else "false"

    @override
    def marshal(self) -> dict[str, str | list[str] | dict[str, Any]]:
        """Convert to a dictionary.

        Overrides the CraftBaseModel to set `exclude_none=True` since `snap sign`
        won't accept empty values.
        """
        return self.model_dump(
            mode="json", by_alias=True, exclude_unset=True, exclude_none=True
        )

    def marshal_scalars_as_strings(self) -> dict[str, Any]:
        """Marshal the model where all scalars are represented as strings."""
        return cast_dict_scalars_to_strings(self.marshal())


Assertion = ConfdbSchemaAssertion | ValidationSetAssertion | ValidationAssertion
EditableAssertion = EditableConfdbSchemaAssertion | EditableValidationSetAssertion
