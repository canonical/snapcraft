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

"""Snapcraft discovery commands."""

from __future__ import annotations

import textwrap
from typing import TYPE_CHECKING

import craft_application.errors
from craft_application.commands import AppCommand
from craft_cli import emit
from craft_parts.plugins import get_registered_plugins
from craft_platforms import DebianArchitecture
from overrides import overrides

from snapcraft import const, errors, models
from snapcraft.parts.yaml_utils import (
    apply_yaml,
    extract_parse_info,
    get_snap_project,
    process_yaml,
)

if TYPE_CHECKING:
    import argparse


class PluginsCommand(AppCommand):
    """List available plugins."""

    name = "plugins"
    help_msg = "List available plugins, optionally for a given base"
    overview = textwrap.dedent(
        """
        List available plugins, optionally for a given base.
        """
    )

    @overrides
    def fill_parser(self, parser: argparse.ArgumentParser) -> None:
        """Add arguments specific to the export-login command."""
        parser.add_argument(
            "--base",
            metavar="base",
            type=str,
            help="Show plugins for <base>",
        )

    @overrides
    def run(self, parsed_args: argparse.Namespace) -> None:
        base = parsed_args.base
        message: str | None = None

        if base in const.ESM_BASES:
            raise errors.MaintenanceBase(base=base)

        if base is None:
            try:
                snap_project = get_snap_project()
                yaml_data = process_yaml(snap_project.project_file)

                # process yaml before unmarshalling the data
                arch = str(DebianArchitecture.from_host())
                yaml_data_for_arch = apply_yaml(yaml_data, arch, arch)
                # discard parse-info as it is not part of Project which we use to
                # determine the base
                extract_parse_info(yaml_data_for_arch)

                project = models.Project.unmarshal(yaml_data_for_arch)
                base = project.get_effective_base()
                message = (
                    f"Displaying plugins available to the current base {base!r} project"
                )
            except craft_application.errors.ProjectFileError:
                emit.trace("Defaulting to core24 because no project was found.")
                base = "core24"

        if message is None:
            message = f"Displaying plugins available for {base!r}"

        if base not in const.CURRENT_BASES:
            raise errors.SnapcraftError(f"{base} not supported")

        registered_plugins = get_registered_plugins()

        emit.message(message + "\n" + "\n".join(n for n in registered_plugins))


class ListPluginsCommand(PluginsCommand):
    """A command alias to list the available plugins."""

    name = "list-plugins"
    hidden = True

    @overrides
    def run(self, parsed_args: argparse.Namespace) -> None:
        emit.progress(
            const.DEPRECATED_COMMAND_WARNING.format(old=self.name, new=super().name),
            permanent=True,
        )
        super().run(parsed_args)
