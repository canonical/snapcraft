# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2019-2022 Canonical Ltd.
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

"""Project model definitions and helpers."""

from typing import TYPE_CHECKING, Any, Dict, List, Literal, Optional

import pydantic
from pydantic import constr

# Workaround for mypy
# see https://github.com/samuelcolvin/pydantic/issues/975#issuecomment-551147305
if TYPE_CHECKING:
    KeyIdStr = str
else:
    KeyIdStr = constr(regex=r"^[0-9A-F]{40}$")


class ProjectModel(pydantic.BaseModel):
    """Base model for project repository classes."""

    class Config:  # pylint: disable=too-few-public-methods
        """Pydantic model configuration."""

        validate_assignment = True
        allow_mutation = False
        allow_population_by_field_name = True
        alias_generator = lambda s: s.replace("_", "-")  # noqa: E731
        extra = "forbid"


# TODO: Project repo definitions are almost the same as PackageRepository
#       ported from legacy. Check if we can consolidate them and remove
#       field validation (moving all validation rules to pydantic).


class AptDeb(ProjectModel):
    """Apt package repository definition."""

    type: Literal["apt"]
    url: str
    key_id: KeyIdStr
    architectures: Optional[List[str]]
    formats: Optional[List[Literal["deb", "deb-src"]]]
    components: Optional[List[str]]
    key_server: Optional[str]
    path: Optional[str]
    suites: Optional[List[str]]

    @classmethod
    def unmarshal(cls, data: Dict[str, Any]) -> "AptDeb":
        """Create an AptDeb object from dictionary data."""
        return cls(**data)


class AptPPA(ProjectModel):
    """PPA package repository definition."""

    type: Literal["apt"]
    ppa: str

    @classmethod
    def unmarshal(cls, data: Dict[str, Any]) -> "AptPPA":
        """Create an AptPPA object from dictionary data."""
        return cls(**data)


def validate_repository(data: Dict[str, Any]):
    """Validate a package repository.

    :param data: The repository data to validate.
    """
    if not isinstance(data, dict):
        raise TypeError("value must be a dictionary")

    try:
        AptPPA(**data)
        return
    except pydantic.ValidationError:
        pass

    AptDeb(**data)
