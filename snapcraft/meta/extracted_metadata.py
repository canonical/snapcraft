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
from typing import List, Optional


@dataclass
class ExtractedMetadata:
    """Collection of metadata extracted from a part."""

    common_id: Optional[str] = None
    """The common identifier across multiple packaging formats."""

    title: Optional[str] = None
    """The extracted package title."""

    summary: Optional[str] = None
    """The extracted package summary."""

    description: Optional[str] = None
    """The extracted package description."""

    version: Optional[str] = None
    """The extracted package version."""

    grade: Optional[str] = None
    """The extracted package version."""

    icon: Optional[str] = None
    """The extracted application icon."""

    desktop_file_paths: List[str] = field(default_factory=list)
    """The extracted application desktop file paths."""
