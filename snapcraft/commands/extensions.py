# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2024 Canonical Ltd.
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

"""Snapcraft extension commands."""

from __future__ import annotations

import textwrap
from typing import TYPE_CHECKING

import tabulate
from craft_application.commands import AppCommand
from craft_cli import emit
from craft_platforms import DebianArchitecture
from overrides import overrides
from pydantic import BaseModel

from snapcraft import errors, extensions, models
from snapcraft.parts.yaml_utils import (
    apply_yaml,
    extract_parse_info,
    get_snap_project,
    process_yaml,
)

if TYPE_CHECKING:
    import argparse


class ExtensionModel(BaseModel):
    """Extension model for presentation."""

    name: str
    bases: list[str]

    def marshal(self) -> dict[str, str]:
        """Marshal model into a dictionary for presentation."""
        return {
            "Extension name": self.name,
            "Supported bases": ", ".join(sorted(self.bases)),
        }


class ExtensionsCommand(AppCommand):
    """List available extensions for all supported bases."""

    name = "extensions"
    help_msg = "List available extensions for all supported bases."
    overview = textwrap.dedent(
        """
        List available extensions and their corresponding bases.
        """
    )

    @overrides
    def run(self, parsed_args: argparse.Namespace) -> None:
        extension_presentation: dict[str, ExtensionModel] = {}

        for extension_name in extensions.registry.get_extension_names():
            extension_class = extensions.registry.get_extension_class(extension_name)
            extension_bases = list(extension_class.get_supported_bases())
            extension_presentation[extension_name] = ExtensionModel(
                name=extension_name, bases=extension_bases
            )

        printable_extensions = sorted(
            [v.marshal() for v in extension_presentation.values()],
            key=lambda d: d["Extension name"],
        )
        emit.message(tabulate.tabulate(printable_extensions, headers="keys"))


class ListExtensionsCommand(ExtensionsCommand):
    """Removed alias to list available extensions."""

    name = "list-extensions"
    hidden = True

    @overrides
    def run(self, parsed_args: argparse.Namespace) -> None:
        raise errors.RemovedCommand(removed_command=self.name, new_command=super().name)


class ExpandExtensionsCommand(AppCommand):
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
    def run(self, parsed_args: argparse.Namespace) -> None:
        """Expand extensions in the project file and output them."""
        snap_project = get_snap_project()
        yaml_data = process_yaml(snap_project.project_file)

        # process yaml before unmarshalling the data
        arch = str(DebianArchitecture.from_host())
        yaml_data_for_arch = apply_yaml(yaml_data, arch, arch)

        # `apply_yaml()` adds or replaces the architectures keyword with an Architecture
        # object, which does not easily dump to a yaml file
        yaml_data_for_arch.pop("architectures", None)
        yaml_data_for_arch.pop("platforms", None)

        # `parse-info` keywords must be removed before unmarshalling, because they are
        # not part of the Project model
        extract_parse_info(yaml_data_for_arch)

        project_data = models.Project.unmarshal(yaml_data_for_arch)
        emit.message(project_data.to_yaml_string())
