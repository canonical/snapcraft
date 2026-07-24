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

"""Model and utilities for component.yaml metadata."""

from pathlib import Path

from craft_application import util

from snapcraft import models
from snapcraft.errors import SnapcraftError

from . import SnapcraftMetadata


class ComponentMetadata(SnapcraftMetadata):
    """The component.yaml model.

    Component hooks are not included in the component's metadata.
    Instead, they are included in the snap's metadata.
    """

    component: str
    type: str
    version: str | None = None
    summary: str
    description: str
    provenance: str | None = None


def get_metadata(project: models.Project, component_name: str) -> ComponentMetadata:
    """Create component metadata for the requested component."""
    if not project.components:
        raise SnapcraftError("Project does not contain any components.")

    component = project.components.get(component_name)

    if not component:
        raise SnapcraftError("Component does not exist.")

    return ComponentMetadata(
        component=f"{project.name}+{component_name}",
        type=component.type,
        version=component.version,
        summary=component.summary,
        description=component.description,
        provenance=project.provenance,
    )


def get_str(project: models.Project, component_name: str) -> str:
    """Create the component.yaml contents for the requested component."""
    return util.dump_yaml(get_metadata(project, component_name).marshal())


def write(
    project: models.Project, component_name: str, component_prime_dir: Path
) -> None:
    """Create a component.yaml file.

    :param project: The snapcraft project.
    :param component_name: Name of the component.
    :param component_prime_dir: The directory containing the component's primed contents.
    """
    meta_dir = component_prime_dir / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)

    meta_dir.joinpath("component.yaml").write_text(
        get_str(project, component_name), encoding="utf-8"
    )
