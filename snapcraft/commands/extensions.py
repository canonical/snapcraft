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
from typing import TYPE_CHECKING, cast

import tabulate
from craft_application.commands import AppCommand
from craft_cli import emit
from pydantic import BaseModel
from typing_extensions import override

from snapcraft import const, errors, extensions, models

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
    """List available extensions, optionally for a given base."""

    name = "extensions"
    help_msg = "List available extensions, optionally for a given base."
    overview = textwrap.dedent(
        """
        List available extensions and their corresponding bases,
        optionally filtered to a given base.
        """
    )

    @override
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        """Add arguments specific to the extensions command."""
        parser.add_argument(
            "--base",
            metavar="base",
            type=str,
            help="Show extensions for <base>",
        )

    @override
    def run(self, parsed_args: argparse.Namespace) -> None:
        base = getattr(parsed_args, "base", None)
        message: str | None = None

        if base is not None:
            if base in const.ESM_BASES:
                raise errors.MaintenanceBase(base=base)
            if base not in const.CURRENT_BASES:
                raise errors.SnapcraftError(f"{base} not supported")
            message = f"Displaying extensions available for {base!r}"

        extension_presentation: dict[str, ExtensionModel] = {}

        for extension_name in extensions.registry.get_extension_names():
            extension_class = extensions.registry.get_extension_class(extension_name)
            extension_bases = list(extension_class.get_supported_bases())
            if base is not None and base not in extension_bases:
                continue
            extension_presentation[extension_name] = ExtensionModel(
                name=extension_name, bases=extension_bases
            )

        printable_extensions = sorted(
            [v.marshal() for v in extension_presentation.values()],
            key=lambda d: d["Extension name"],
        )
        table = tabulate.tabulate(printable_extensions, headers="keys")
        if message is None:
            emit.message(table)
        else:
            emit.message(message + "\n" + table)


class ListExtensionsCommand(ExtensionsCommand):
    """Removed alias to list available extensions."""

    name = "list-extensions"
    hidden = True

    @override
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

    always_load_project = True

    @override
    def run(self, parsed_args: argparse.Namespace) -> None:
        """Expand extensions in the project file and output them."""
        project = cast(models.Project, self._services.get("project").get())
        emit.message(project.to_yaml_string())
