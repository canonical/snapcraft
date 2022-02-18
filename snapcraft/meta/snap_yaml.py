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

"""Create snap.yaml metadata file."""

import textwrap
from pathlib import Path
from typing import Any, Dict, List, Optional, cast

import yaml
from pydantic_yaml import YamlModel

from snapcraft.projects import Project


class SnapApp(YamlModel):
    """Snap.yaml app entry.

    This is currently a partial implementation, see
    https://snapcraft.io/docs/snap-format for details.

    TODO: add missing properties, improve validation (CRAFT-802)
    """

    command: str
    command_chain: List[str]
    environment: Optional[Dict[str, Any]]
    plugs: Optional[List[str]]

    class Config:  # pylint: disable=too-few-public-methods
        """Pydantic model configuration."""

        allow_population_by_field_name = True
        alias_generator = lambda s: s.replace("_", "-")  # noqa: E731


class SnapMetadata(YamlModel):
    """The snap.yaml model.

    This is currently a partial implementation, see
    https://snapcraft.io/docs/snap-format for details.

    TODO: add missing properties, improve validation (CRAFT-802)
    """

    name: str
    title: Optional[str]
    version: str
    summary: str
    description: str
    license: Optional[str]
    type: str
    architectures: List[str]
    base: str
    assumes: Optional[List[str]]
    epoch: Optional[str]
    apps: Optional[Dict[str, SnapApp]]
    confinement: str
    grade: str
    environment: Optional[Dict[str, Any]]


def write(project: Project, prime_dir: Path, *, arch: str):
    """Create a snap.yaml file."""
    meta_dir = prime_dir / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)

    _write_snapcraft_runner(prime_dir)

    snap_apps: Dict[str, SnapApp] = {}
    if project.apps:
        for name, app in project.apps.items():
            snap_apps[name] = SnapApp(
                command=app.command,
                command_chain=["snap/command-chain/snapcraft-runner"],
                environment=app.environment,
                plugs=app.plugs,
            )

    # FIXME: handle adopted parameters
    snap_metadata = SnapMetadata(
        name=project.name,
        title=project.title,
        version=project.version,  # type: ignore
        summary=project.summary,
        description=project.description,
        license=project.license,
        type=project.type,
        architectures=[arch],
        base=cast(str, project.base),
        assumes=["command-chain"],
        epoch=project.epoch,
        apps=snap_apps,
        confinement=project.confinement,
        grade=project.grade,
        environment=project.environment,
    )

    yaml.add_representer(str, _repr_str, Dumper=yaml.SafeDumper)
    yaml_data = snap_metadata.yaml(exclude_none=True, sort_keys=False, width=1000)

    snap_yaml = meta_dir / "snap.yaml"
    snap_yaml.write_text(yaml_data)


def _write_snapcraft_runner(prime_dir: Path):
    content = textwrap.dedent(
        """#!/bin/sh
        export PATH="$SNAP/usr/sbin:$SNAP/usr/bin:$SNAP/sbin:$SNAP/bin:$PATH"
        export LD_LIBRARY_PATH=$SNAP_LIBRARY_PATH:$LD_LIBRARY_PATH
        exec "$@"
        """
    )

    runner_path = prime_dir / "snap/command-chain/snapcraft-runner"
    runner_path.parent.mkdir(parents=True, exist_ok=True)
    runner_path.write_text(content)
    runner_path.chmod(0o755)


def _repr_str(dumper, data):
    """Multi-line string representer for the YAML dumper."""
    if "\n" in data:
        return dumper.represent_scalar("tag:yaml.org,2002:str", data, style="|")
    return dumper.represent_scalar("tag:yaml.org,2002:str", data)
