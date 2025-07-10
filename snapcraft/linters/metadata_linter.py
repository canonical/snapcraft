# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
"""Snapcraft metadata linter.

Validates that the snap's ``snapcraft.yaml`` (represented by ``SnapMetadata``)
contains the minimum viable set of metadata fields and that their values look
sane.  The checks are grouped into *levels* so the user can prioritise the
highest-impact fixes first.
"""

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Optional

from overrides import overrides

from snapcraft.meta.snap_yaml import SnapMetadata

from .base import Linter, LinterIssue, LinterResult

_HELP_URL = "https://documentation.ubuntu.com/snapcraft/stable/reference/project-file/snapcraft-yaml/"


@dataclass
class MetadataField:
    name: str
    severity: LinterResult
    extract: Callable[["SnapMetadata"], Any]
    help_url: str
    skip: Callable[["SnapMetadata"], bool] = lambda _: False


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
    MetadataField(
        "common_id",
        LinterResult.WARNING,
        lambda meta: _get_apps_attr(meta, "common_id"),
        f"{_HELP_URL}#apps.<app-name>.common-id",
        lambda meta: bool((meta.type and meta.type != "app") or not meta.apps),
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
        "source_code",
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
    def _is_dict_empty(
        cls, value: dict[str, list[str]] | None
    ) -> dict[str, bool] | bool:
        """Check if dictionary values are empty or None.

        :param value: Dictionary to check

        :returns: True if all values are empty, Dict mapping keys to empty status otherwise
        """
        if value is None:
            return True
        result = {}
        for key, values in value.items():
            if not values:
                result[key] = True

        return result if result else False

    @classmethod
    def _is_empty(
        cls,
        value: str | dict[str, list[str]] | list[str] | None,
    ) -> dict[str, bool] | bool:
        """Check if a value is empty, handling different types appropriately.

        :param value: The value to check

        :returns: True if empty, False if not empty, or Dict for partial emptiness in dict values
        """
        if value is None:
            return True
        if isinstance(value, str):
            return value.strip() == ""
        if isinstance(value, list) and not value:
            return True
        if isinstance(value, dict):
            return cls._is_dict_empty(value)
        return False

    def _create_issue(
        self, field: MetadataField, text: str, result: Optional["LinterResult"] = None
    ) -> "LinterIssue":
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

    def _check_field(self, field: MetadataField) -> list["LinterIssue"]:
        """Check if a metadata field is complete and create issues if not.

        :param field: The metadata field to check

        :returns: List of linter issues found
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
