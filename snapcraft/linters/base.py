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
import fnmatch
from pathlib import Path
from typing import TYPE_CHECKING, List, Literal, Optional, Union

import pydantic
from craft_cli import emit

from snapcraft import projects
from snapcraft.elf import ElfFile

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
    url: Optional[str]
    suggested_changes: Optional[str]  # XXX: pending definition

    def __init__(self, **kwargs):
        super().__init__(type="lint", **kwargs)

    def __str__(self):
        """Use short formatted issue as the string representation."""
        if self.filename:
            msg = f"{self.name!s}: {self.filename}: {self.text}"
        else:
            msg = f"{self.name!s}: {self.text}"

        if self.url:
            msg += f" ({self.url})"

        return msg

    class Config:
        """Pydantic model configuration."""

        validate_assignment = True
        extra = "forbid"
        alias_generator = lambda s: s.replace("_", "-")  # noqa: E731


class Linter(abc.ABC):
    """Base class for linters.

    :param project: The snap project information.
    """

    def __init__(
        self,
        name: str,
        snap_metadata: "SnapMetadata",
        lint: Optional[projects.Lint],
    ):
        self._name = name
        self._snap_metadata = snap_metadata
        self._lint = lint or projects.Lint(ignore=[])

    @abc.abstractmethod
    def run(self) -> List[LinterIssue]:
        """Execute linting.

        :return: A list of linter issues flagged by this linter.
        """

    def _is_file_ignored(
        self, filepath: Union[ElfFile, Path], category: str = ""
    ) -> bool:
        """Check if the file name matches an ignored file pattern.

        :param category:
            For linters that implement categories, the specific category that
            should be checked for the ignored patterns. `filepath` will be checked
            against the linter's "main" patterns (based on the linter's name) *and*
            against the category-specific patterns.
        """
        ignored_files = self._lint.ignored_files(self._name)

        if category:
            # No "extend()" because we don't want to affect the original list.
            ignored_files = ignored_files + self._lint.ignored_files(category)

        if isinstance(filepath, ElfFile):
            path = filepath.path
        else:
            path = filepath

        for pattern in ignored_files:
            if fnmatch.fnmatch(str(path), pattern):
                emit.debug(
                    f"{self._name} linter: skip file {str(path)!r} "
                    f"(matches {pattern!r})"
                )
                return True

        return False

    @staticmethod
    def get_categories() -> List[str]:
        """Get a list of specific subcategories that can be filtered against.

        For Linter subclasses that perform multiple "kinds" of linting, this
        must return the list of names that can be used to selectively ignore
        parts of the linting behavior for given paths.
        """
        return []
