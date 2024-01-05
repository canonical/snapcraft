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
from typing import Any, Dict, Optional

import yaml
from pydantic import Extra
from pydantic_yaml import YamlModel

from snapcraft.errors import SnapcraftError
from snapcraft.projects import Project


class ComponentMetadata(YamlModel):
    """The component.yaml model."""

    component: str
    type: str
    version: Optional[str]
    summary: str
    description: str

    class Config:  # pylint: disable=too-few-public-methods
        """Pydantic model configuration."""

        allow_population_by_field_name = True
        alias_generator = lambda s: s.replace("_", "-")  # noqa: E731
        extra = Extra.forbid

    @classmethod
    def unmarshal(cls, data: Dict[str, Any]) -> "ComponentMetadata":
        """Create and populate a new ``ComponentMetadata`` object from dictionary data.

        The unmarshal method validates entries in the input dictionary, populating
        the corresponding fields in the data object.

        :param data: The dictionary data to unmarshal.

        :return: The newly created object.

        :raise TypeError: If data is not a dictionary.
        """
        if not isinstance(data, dict):
            raise TypeError("data is not a dictionary")

        return cls(**data)


def write(project: Project, component_name: str, component_prime_dir: Path) -> None:
    """Create a component.yaml file.

    :param project: The snapcraft project.
    :param component_name: Name of the component.
    :param component_prime_dir: The directory containing the component's primed contents.
    """
    meta_dir = component_prime_dir / "meta"
    meta_dir.mkdir(parents=True, exist_ok=True)

    if not project.components:
        raise SnapcraftError("Project does not contain any components.")

    component = project.components.get(component_name)

    if not component:
        raise SnapcraftError("Component does not exist.")

    component_metadata = ComponentMetadata(
        component=f"{project.name}+{component_name}",
        type=component.type,
        version=component.version,
        summary=component.summary,
        description=component.description,
    )

    yaml.add_representer(str, _repr_str, Dumper=yaml.SafeDumper)
    yaml_data = component_metadata.yaml(
        by_alias=True,
        exclude_none=True,
        allow_unicode=True,
        sort_keys=False,
        width=1000,
    )

    component_yaml = meta_dir / "component.yaml"
    component_yaml.write_text(yaml_data)


def _repr_str(dumper, data):
    """Multi-line string representer for the YAML dumper."""
    if "\n" in data:
        return dumper.represent_scalar("tag:yaml.org,2002:str", data, style="|")
    return dumper.represent_scalar("tag:yaml.org,2002:str", data)
