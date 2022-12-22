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

"""Define and create the manifest file."""

import json
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

from pydantic_yaml import YamlModel

from snapcraft import __version__, errors, os_release, utils
from snapcraft.projects import Project


class Manifest(YamlModel):
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
    base: Optional[str]
    grade: str
    confinement: str
    apps: Optional[Dict[str, Any]]
    parts: Dict[str, Any]

    # TODO: add assumes, environment, hooks, slots

    # Architecture
    architectures: List[str]

    # Image info
    image_info: Dict[str, Any]

    # Build environment
    build_packages: List[str]
    build_snaps: List[str]
    primed_stage_packages: List

    class Config:  # pylint: disable=too-few-public-methods
        """Pydantic model configuration."""

        allow_population_by_field_name = True
        alias_generator = lambda s: s.replace("_", "-")  # noqa: E731


def write(
    project: Project,
    prime_dir: Path,
    *,
    arch: str,
    parts: Dict[str, Any],
    image_information: str,
    start_time: datetime,
    primed_stage_packages: List[str],
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

    yaml_data = manifest.yaml(
        by_alias=True,
        exclude_none=True,
        exclude_unset=True,
        allow_unicode=True,
        sort_keys=False,
        width=1000,
    )

    manifest_yaml = snap_dir / "manifest.yaml"
    manifest_yaml.write_text(yaml_data)
