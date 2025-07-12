# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
"""Snapcraft metadata linter.

Validates that the snap's ``snapcraft.yaml`` (represented by ``SnapMetadata``)
contains the minimum viable set of metadata fields and that their values look
sane.  The checks are grouped into *ranks* so the user can prioritise the
highest‑impact fixes first.

Rank‑to‑severity mapping
========================
* **Rank‑1** – *Fatal* (build fails)
* **Rank‑2** – *Error* (build continues but exits non‑zero)
* **Rank‑3** – *Warning* (informational)
"""

import enum
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Optional

from overrides import overrides

from snapcraft.meta.snap_yaml import SnapMetadata

from .base import Linter, LinterIssue, LinterResult

_HELP_URL = "https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/"


class Rank(enum.IntEnum):
    """Relative importance of a metadata field."""

    RANK1 = 1
    RANK2 = 2

    @property
    def severity(self) -> LinterResult:
        if self is Rank.RANK1:
            return LinterResult.WARNING
        return LinterResult.INFO


@dataclass
class MetadataField:
    name: str
    rank: Rank
    extract: Callable[["SnapMetadata"], Any]
    lint: bool = True


# Helper to pull the matching attribute values from all the apps
def _get_apps_attr(meta: SnapMetadata, key: str) -> dict[str, list[str]] | None:
    if not meta.apps:
        return None

    values: dict[str, list[str]] = {}
    for name, app in meta.apps.items():
        value: str | list[str] | None = getattr(app, key, None)
        if value and isinstance(value, list):
            values[name] = value
        elif isinstance(value, str):
            values[name] = [value]
        else:
            values[name] = []

    return values if values else None


# Helper to pull the matching attribute values from the links
def _get_links_attr(meta: SnapMetadata, key: str) -> list[str] | str | None:
    if not meta.links:
        return None

    return getattr(meta.links, key, None)


_FIELDS: list[MetadataField] = [
    # TODO: implement desktop field in Rank1
    # TODO: implement icon field in Rank1
    # Rank 1 fields
    MetadataField("title", Rank.RANK1, lambda meta: meta.title),
    MetadataField("contact", Rank.RANK1, lambda meta: _get_links_attr(meta, "contact")),
    MetadataField("license", Rank.RANK1, lambda meta: meta.license),
    MetadataField(
        "common_id", Rank.RANK1, lambda meta: _get_apps_attr(meta, "common_id")
    ),
    # Rank 2 fields
    MetadataField(
        "donation", Rank.RANK2, lambda meta: _get_links_attr(meta, "donation")
    ),
    MetadataField("issues", Rank.RANK2, lambda meta: _get_links_attr(meta, "issues")),
    MetadataField(
        "source_code", Rank.RANK2, lambda meta: _get_links_attr(meta, "source_code")
    ),
    MetadataField("website", Rank.RANK2, lambda meta: _get_links_attr(meta, "website")),
]


class MetadataLinter(Linter):
    """Checks snap metadata completeness and semantic validity."""

    @staticmethod
    def get_categories() -> list[str]:
        return [field.name for field in _FIELDS]

    @staticmethod
    def _is_dict_empty(value: dict[str, list[str]] | None) -> dict[str, bool] | bool:
        """Check if dictionary values are empty or None.

        Args:
            value: Dictionary to check

        Returns:
            True if all values are empty, Dict mapping keys to empty status otherwise
        """
        if value is None:
            return True
        for key, values in value.items():
            if not values:
                return {key: True}
        return False

    @staticmethod
    def _is_empty(
        value: str | dict[str, list[str]] | list[str] | None,
    ) -> dict[str, bool] | bool:
        """Check if a value is empty, handling different types appropriately.

        Args:
            value: The value to check

        Returns:
            True if empty, False if not empty, or Dict for partial emptiness in dict values
        """
        if value is None:
            return True
        if isinstance(value, str):
            return value.strip() == ""
        if isinstance(value, list) and not value:
            return True
        if isinstance(value, dict):
            return MetadataLinter._is_dict_empty(value)
        return False

    def _create_issue(
        self, field: MetadataField, text: str, result: Optional["LinterResult"] = None
    ) -> "LinterIssue":
        """Create a linter issue for a metadata field.

        Args:
            field: The metadata field
            text: Issue description
            result: Override result type (defaults to field rank severity)

        Returns:
            LinterIssue instance
        """
        field_name = field.name.replace("_", "-")
        help_url = f"{_HELP_URL}#{field_name}"

        return LinterIssue(
            name=self._name,
            result=result or field.rank.severity,
            text=text,
            url=help_url if result is not LinterResult.IGNORED else None,
        )

    def _check_field(self, field: MetadataField) -> list["LinterIssue"]:
        """Check if a metadata field is complete and create issues if not.

        Args:
            field: The metadata field to check

        Returns:
            List of linter issues found
        """
        issues = []
        value = field.extract(self._snap_metadata)
        field_name = field.name.replace("_", "-")

        result = self._is_empty(value)

        if isinstance(result, bool) and result:
            issues.append(
                self._create_issue(
                    field,
                    f"Metadata field '{field_name}' is missing or empty.",
                )
            )
        elif isinstance(result, dict):
            for name, is_empty in result.items():
                if is_empty:
                    issues.append(
                        self._create_issue(
                            field,
                            f"Metadata field '{field_name}' for app '{name}' is missing or empty.",
                        )
                    )

        return issues

    @overrides
    def run(self) -> list[LinterIssue]:
        meta: SnapMetadata = self._snap_metadata

        if meta.grade == "devel":
            return []

        issues: list[LinterIssue] = []
        for field in _FIELDS:
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
