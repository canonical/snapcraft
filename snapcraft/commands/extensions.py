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

import textwrap
from typing import Dict, List, cast

import tabulate
from craft_application.commands import AppCommand
from craft_cli import emit
from overrides import overrides
from pydantic import BaseModel

from snapcraft import extensions, models
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


class ListExtensionsCommand(AppCommand):
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


class ExtensionsCommand(ListExtensionsCommand):
    """A command alias to list the available extensions."""

    name = "extensions"
    hidden = True


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

    always_load_project = True

    @overrides
    def run(self, parsed_args):
        project = cast(models.Project, self._services.project)
        emit.message(project.to_yaml_string())
