# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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

"""Manifest file definition."""

from typing import Any

from craft_application.models import BaseMetadata


class Manifest(BaseMetadata):
    """Manifest file for snaps."""

    # Snapcraft annotations
    snapcraft_version: str
    snapcraft_started_at: str
    snapcraft_os_release_id: str
    snapcraft_os_release_version_id: str

    # Project fields
    name: str
    version: str
    summary: str
    description: str
    base: str | None
    grade: str
    confinement: str
    apps: dict[str, Any] | None
    parts: dict[str, Any]

    # TODO: add assumes, environment, hooks, slots

    # Architecture
    architectures: list[str]

    # Image info
    image_info: dict[str, Any]

    # Build environment
    build_packages: list[str]
    build_snaps: list[str]
    primed_stage_packages: list
