# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2017-2022 Canonical Ltd.
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

"""External metadata definition."""

from dataclasses import dataclass, field


@dataclass
class ExtractedMetadata:
    """Collection of metadata extracted from a part."""

    common_id: str | None = None
    """The common identifier across multiple packaging formats."""

    title: str | None = None
    """The extracted package title."""

    summary: str | None = None
    """The extracted package summary."""

    description: str | None = None
    """The extracted package description."""

    version: str | None = None
    """The extracted package version."""

    grade: str | None = None
    """The extracted package version."""

    icon: str | None = None
    """The extracted application icon."""

    desktop_file_paths: list[str] = field(default_factory=list)
    """The extracted application desktop file paths."""

    license: str | None = None
    """The extracted package license"""

    contact: str | None = None
    """The extracted package contact"""

    donation: list[str] | None = None
    """The extracted package donation"""

    issues: list[str] | None = None
    """The extracted package issues"""

    source_code: str | None = None
    """The extracted package source code"""

    website: list[str] | None = None
    """The extracted package website"""
