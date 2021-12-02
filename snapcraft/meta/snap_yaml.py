# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2021 Canonical Ltd.
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

"""Lifecycle integration."""

from pathlib import Path
from typing import Any, Dict, List

from pydantic_yaml import YamlModel  # type: ignore

from snapcraft.projects import Project


class SnapApp(YamlModel):
    command: str
    command_chain: List[str]

    class Config:  # pylint: disable=too-few-public-methods
        """Pydantic model configuration."""

        allow_population_by_field_name = True
        alias_generator = lambda s: s.replace("_", "-")  # noqa: E731


class SnapMetadata(YamlModel):
    name: str
    version: str
    summary: str
    description: str
    architectures: List[str]
    base: str
    apps: Dict[str, SnapApp]
    assumes: List[str]
    confinement: str
    grade: str


def write(project: Project, prime_dir: Path, *, arch: str):
    """Create a bare minimum snap.yaml for the new-snapcraft spike."""
    meta = prime_dir / "meta"
    meta.mkdir(exist_ok=True)

    snap_apps: Dict[str, SnapApp] = {}
    for name, app in project.apps.items():
        snap_apps[name] = SnapApp(
            command=app.command, command_chain=["snap/command-chain/snapcraft-runner"]
        )

    snap_metadata = SnapMetadata(
        name=project.name,
        version=project.version,
        summary=project.summary,
        description=project.description,
        architectures=[arch],
        base=project.base,
        apps=snap_apps,
        assumes=["command-chain"],
        confinement=project.confinement,
        grade=project.grade,
    )

    yaml_data = snap_metadata.yaml()

    snap_yaml = meta / "snap.yaml"
    snap_yaml.write_text(yaml_data)
