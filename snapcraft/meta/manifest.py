# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022,2024 Canonical Ltd.
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

"""Define and create the manifest file."""

import json
from datetime import datetime
from pathlib import Path
from typing import Any

import craft_application.models

from snapcraft import __version__, errors, models, os_release, utils


class Manifest(craft_application.models.CraftBaseModel):
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
    base: str | None = None
    grade: str
    confinement: str
    apps: dict[str, Any] | None = None
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


def write(  # noqa PLR0913
    project: models.Project,
    prime_dir: Path,
    *,
    arch: str,
    parts: dict[str, Any],
    image_information: str,
    start_time: datetime,
    primed_stage_packages: list[str],
):
    """Create a manifest.yaml file."""
    snap_dir = prime_dir / "snap"
    snap_dir.mkdir(parents=True, exist_ok=True)

    osrel = os_release.OsRelease()
    version = utils.process_version(project.version)

    try:
        image_info = json.loads(image_information)
    except json.decoder.JSONDecodeError as err:
        raise errors.SnapcraftError(
            f"Image information decode error at {err.lineno}:{err.colno}: "
            f"{err.doc!r}: {err.msg}"
        ) from err

    manifest = Manifest(
        # Snapcraft annotations
        snapcraft_version=__version__,
        snapcraft_started_at=start_time.isoformat("T") + "Z",
        snapcraft_os_release_id=osrel.name().lower(),
        snapcraft_os_release_version_id=osrel.version_id().lower(),
        # Project fields
        name=project.name,
        version=version,
        summary=project.summary,  # type: ignore
        description=project.description,  # type: ignore
        base=project.base,
        grade=project.grade or "stable",
        confinement=project.confinement,
        apps=project.apps,
        parts=parts,
        # Architecture
        architectures=[arch],
        # Image info
        image_info=image_info,
        # Build environment
        build_packages=[],
        build_snaps=[],
        primed_stage_packages=primed_stage_packages,
    )

    manifest.to_yaml_file(snap_dir / "manifest.yaml")
