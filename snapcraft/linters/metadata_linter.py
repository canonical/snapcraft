# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2025 Canonical Ltd.
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

"""Metadata linter implementation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import craft_cli
from overrides import overrides

from .base import Linter, LinterIssue, LinterResult

if TYPE_CHECKING:
    from collections.abc import Callable

    from snapcraft.meta.snap_yaml import SnapMetadata

_HELP_URL = "https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/"


@dataclass
class MetadataField:
    name: str
    severity: LinterResult
    extract: Callable[[SnapMetadata], Any]
    help_url: str
    skip: Callable[[SnapMetadata], bool] = lambda _: False


def _get_links_attr(meta: SnapMetadata, key: str) -> list[str] | str | None:
    """
    Get the value of a links attribute.

    :param meta: The snap metadata
    :param key: The attribute name

    :returns: The value of the attribute
    """
    if not meta.links:
        return None

    return getattr(meta.links, key, None)


_FIELDS: list[MetadataField] = [
    # Rank 1 fields
    MetadataField(
        "title", LinterResult.WARNING, lambda meta: meta.title, f"{_HELP_URL}#title"
    ),
    MetadataField(
        "contact",
        LinterResult.WARNING,
        lambda meta: _get_links_attr(meta, "contact"),
        f"{_HELP_URL}#contact",
    ),
    MetadataField(
        "license",
        LinterResult.WARNING,
        lambda meta: meta.license,
        f"{_HELP_URL}#license",
    ),
    # Rank 2 fields
    MetadataField(
        "donation",
        LinterResult.INFO,
        lambda meta: _get_links_attr(meta, "donation"),
        f"{_HELP_URL}#donation",
    ),
    MetadataField(
        "issues",
        LinterResult.INFO,
        lambda meta: _get_links_attr(meta, "issues"),
        f"{_HELP_URL}#issues",
    ),
    MetadataField(
        "source-code",
        LinterResult.INFO,
        lambda meta: _get_links_attr(meta, "source_code"),
        f"{_HELP_URL}#source-code",
    ),
    MetadataField(
        "website",
        LinterResult.INFO,
        lambda meta: _get_links_attr(meta, "website"),
        f"{_HELP_URL}#website",
    ),
]


class MetadataLinter(Linter):
    """Checks snap metadata completeness and semantic validity."""

    @staticmethod
    def get_categories() -> list[str]:
        return [field.name for field in _FIELDS]

    @classmethod
    def _is_empty(
        cls,
        value: str | dict[str, list[str]] | list[str] | None,
    ) -> bool:
        """Check if a value is empty, handling different types appropriately.

        :param value: The value to check

        :returns: True if empty, False if not empty
        """
        if value is None:
            return True
        if isinstance(value, str):
            return value.strip() == ""
        if isinstance(value, list) and not value:
            return True
        return False

    def _create_issue(
        self, field: MetadataField, text: str, result: LinterResult | None = None
    ) -> LinterIssue:
        """Create a linter issue for a metadata field.

        :param field: The metadata field
        :param text: Issue description
        :param result: Override result type (defaults to field rank severity)

        :returns: LinterIssue instance
        """
        help_url = field.help_url if result is not LinterResult.IGNORED else None

        return LinterIssue(
            name=self._name,
            result=result or field.severity,
            text=text,
            url=help_url,
        )

    def _check_field(self, field: MetadataField) -> list[LinterIssue]:
        """Check if a metadata field is complete and create issues if not.

        :param field: The metadata field to check

        :returns: List of linter issues found
        """
        value = field.extract(self._snap_metadata)
        field_name = field.name.replace("_", "-")

        if not self._is_empty(value):
            return []

        return [
            self._create_issue(
                field,
                f"Metadata field '{field_name}' is missing or empty.",
            )
        ]

    @overrides
    def run(self) -> list[LinterIssue]:
        meta: SnapMetadata = self._snap_metadata

        if meta.grade == "devel":
            craft_cli.emit.debug("Skipping metadata linter because grade is 'devel'.")
            return []

        issues: list[LinterIssue] = []
        for field in _FIELDS:
            if field.skip(meta):
                continue

            if self._lint and field.name in self._lint.ignored_files(self._name):
                issues.append(
                    self._create_issue(
                        field,
                        f"Metadata field '{field.name}' is ignored.",
                        LinterResult.IGNORED,
                    )
                )
                continue

            issues.extend(self._check_field(field))

        return issues
