# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2023 Canonical Ltd.
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

"""Snapcraft lifecycle commands."""

import abc
import textwrap
from typing import Dict, List

import tabulate
import yaml
from craft_cli import BaseCommand, emit
from overrides import overrides
from pydantic import BaseModel

from snapcraft import extensions, models
from snapcraft.parts.yaml_utils import (
    apply_yaml,
    extract_parse_info,
    get_snap_project,
    process_yaml,
)
from snapcraft.utils import get_host_architecture
from snapcraft_legacy.internal.project_loader import (
    find_extension,
    supported_extension_names,
)


class ExtensionModel(BaseModel):
    """Extension model for presentation."""

    name: str
    bases: List[str]

    def marshal(self) -> Dict[str, str]:
        """Marshal model into a dictionary for presentation."""
        return {
            "Extension name": self.name,
            "Supported bases": ", ".join(sorted(self.bases)),
        }


class ListExtensionsCommand(BaseCommand, abc.ABC):
    """List available extensions for all supported bases."""

    name = "list-extensions"
    help_msg = "List available extensions for all supported bases."
    overview = textwrap.dedent(
        """
        List available extensions and their corresponding bases.
        """
    )

    @overrides
    def run(self, parsed_args):
        extension_presentation: Dict[str, ExtensionModel] = {}

        # New extensions.
        for extension_name in extensions.registry.get_extension_names():
            extension_class = extensions.registry.get_extension_class(extension_name)
            extension_bases = list(extension_class.get_supported_bases())
            extension_presentation[extension_name] = ExtensionModel(
                name=extension_name, bases=extension_bases
            )

        # Extensions from snapcraft_legacy.
        for _extension_name in supported_extension_names():
            extension_class = find_extension(_extension_name)
            extension_name = _extension_name.replace("_", "-")
            extension_bases = list(extension_class.get_supported_bases())
            if extension_name in extension_presentation:
                extension_presentation[extension_name].bases += extension_bases
            else:
                extension_presentation[extension_name] = ExtensionModel(
                    name=extension_name, bases=extension_bases
                )

        printable_extensions = sorted(
            [v.marshal() for v in extension_presentation.values()],
            key=lambda d: d["Extension name"],
        )
        emit.message(tabulate.tabulate(printable_extensions, headers="keys"))


class ExtensionsCommand(ListExtensionsCommand, abc.ABC):
    """A command alias to list the available extensions."""

    name = "extensions"
    hidden = True


class ExpandExtensionsCommand(BaseCommand, abc.ABC):
    """Expand the extensions in the snapcraft.yaml file."""

    name = "expand-extensions"
    help_msg = "Expand extensions in snapcraft.yaml"
    overview = textwrap.dedent(
        """
        Extensions selected in apps in snapcraft.yaml will be
        expanded and shown as output.
        """
    )

    @overrides
    def run(self, parsed_args):
        snap_project = get_snap_project()

        # load yaml file and trigger legacy behavior if base is core, core18, or core20
        yaml_data = process_yaml(snap_project.project_file)

        # process yaml before unmarshalling the data
        arch = get_host_architecture()
        yaml_data_for_arch = apply_yaml(yaml_data, arch, arch)

        # `apply_yaml()` adds or replaces the architectures keyword with an Architecture
        # object, which does not easily dump to a yaml file
        yaml_data_for_arch.pop("architectures")

        # `parse-info` keywords must be removed before unmarshalling, because they are
        # not part of the Project model
        extract_parse_info(yaml_data_for_arch)

        models.Project.unmarshal(yaml_data_for_arch)
        emit.message(yaml.safe_dump(yaml_data_for_arch, indent=4, sort_keys=False))
