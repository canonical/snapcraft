# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2021,2024 Canonical Ltd
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

from typing import Any, Annotated, Dict, Literal, TYPE_CHECKING
import pydantic

from craft_application import models


if TYPE_CHECKING:
    SnapName = str
    SnapId = str
else:
    SnapName = pydantic.constr(max_length=40)
    SnapId = pydantic.constr(max_length=40)


def cast_dict_values_to_strings(data: dict[str, Any]) -> dict[str, Any]:
    """Cast all scalars in a dictionary to strings."""
    return {k: _to_string(v) for k, v in data.items()}


def _to_string(data: list | str | int | None) -> dict | list | str | None:
    """Recurse through nested lists and cast all scalars to strings."""
    if isinstance(data, dict):
        return {k: _to_string(v) for k, v in data.items()}
    elif isinstance(data, list):
        return [_to_string(i) for i in data]
    elif data:
        return str(data)
    return data


class Snap(models.CraftBaseModel):
    """Represent a Snap in a Validation Set."""

    name: SnapName
    """Snap name"""

    id: SnapId | None
    """Snap ID"""

    presence: Literal["required", "optional", "invalid"] | None
    """Snap presence"""

    revision: int | None
    """Snap revision"""


class EditableBuildAssertion(models.CraftBaseModel):
    """Subset of a build assertion that can be edited by the user.

    https://dashboard.snapcraft.io/docs/reference/v2/en/validation-sets.html#request-json-schema
    """
    account_id: str
    """The "account-id" assertion header"""

    name: str
    """The "name" assertion header"""

    revision: str | None
    """The "revision" assertion header"""

    sequence: int
    """The "sequence" assertion header"""

    snaps: Annotated[list[Snap], pydantic.Field(min_items=1)]
    """List of snaps in a Validation Set assertion"""

    def marshal_as_str(self) -> Dict[str, Any]:
        """Marshal the object where all scalars are represented as strings."""
        data = self.marshal()
        return cast_dict_values_to_strings(data)


class BuildAssertion(EditableBuildAssertion):
    """Full build assertion header for a Validation Set.

    https://dashboard.snapcraft.io/docs/reference/v2/en/validation-sets.html#response-json-schema
    """
    authority_id: str
    """The "authority-id" assertion header"""

    series: str
    """The "series" assertion header"""

    sign_key_sha3_384: str | None
    """Signing key ID."""

    timestamp: str
    """The "timestamp" assertion header"""

    type: Literal["validation-set"]
    """The "type" assertion header"""

    @pydantic.root_validator(pre=True)
    def remove_sign_key(cls, values):
        """Accept but always ignore the sign key.

        The API can return and accept the sign key but the previous implementation
        ignored it.
        """
        values.pop("sign-key-sha3-384", None)
        return values


class Headers(models.CraftBaseModel):
    """Assertion headers for a validation set."""
    headers: BuildAssertion
    """Assertion headers"""


class ValidationSets(models.CraftBaseModel):
    """Validation sets.

    https://dashboard.snapcraft.io/docs/reference/v2/en/validation-sets.html#id4
    """
    assertions: list[Headers]
    """List of validation-set assertions"""
