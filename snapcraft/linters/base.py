# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

"""Snapcraft linting infrastructure."""

import abc
import enum
from typing import TYPE_CHECKING, List, Literal, Optional

import pydantic

from snapcraft import projects

if TYPE_CHECKING:
    from snapcraft.meta.snap_yaml import SnapMetadata


@enum.unique
class LinterResult(str, enum.Enum):
    """Valid linting results."""

    OK = "ok"
    WARNING = "warning"
    ERROR = "error"
    FATAL = "fatal"
    IGNORED = "ignored"

    def __str__(self):
        """Use the enum value as the string representation."""
        return self.value


class LinterIssue(pydantic.BaseModel):
    """Linter issue with type "lint"."""

    type: Literal["lint"]
    name: str
    result: LinterResult
    filename: Optional[str]
    text: str
    url: str
    suggested_changes: Optional[str]  # XXX: pending definition

    def __init__(self, **kwargs):
        super().__init__(type="lint", **kwargs)

    def __str__(self):
        """Use short formatted issue as the string representation."""
        if not self.filename:
            return f"{self.name!s}: {self.text} ({self.url})"

        return f"{self.name!s}: {self.filename}: {self.text} ({self.url})"

    class Config:  # pylint: disable=too-few-public-methods
        """Pydantic model configuration."""

        validate_assignment = True
        extra = "forbid"
        alias_generator = lambda s: s.replace("_", "-")  # noqa: E731


class Linter(abc.ABC):
    """Base class for linters.

    :param project: The snap project information.
    """

    def __init__(
        self, name: str, snap_metadata: "SnapMetadata", lint: Optional[projects.Lint]
    ):
        self._name = name
        self._snap_metadata = snap_metadata
        self._lint = lint or projects.Lint(ignore=projects.LintIgnore())

    @abc.abstractmethod
    def run(self) -> List[LinterIssue]:
        """Execute linting.

        :return: A list of linter issues flagged by this linter.
        """
