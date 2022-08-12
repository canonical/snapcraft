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

"""Snapcraft lifecycle commands."""

import abc
import textwrap
from typing import TYPE_CHECKING, Optional

from craft_cli import BaseCommand, emit
from craft_parts.plugins import get_registered_plugins
from overrides import overrides

from snapcraft import errors
from snapcraft.parts.lifecycle import get_snap_project, process_yaml
from snapcraft.projects import Project

if TYPE_CHECKING:
    import argparse


class ListPluginsCommand(BaseCommand, abc.ABC):
    """A command to list the available extensions."""

    name = "list-plugins"
    help_msg = "List the available plugins that handle different types of part"
    overview = textwrap.dedent(
        """
        List the available plugins that handle different types of part
        """
    )

    @overrides
    def fill_parser(self, parser: "argparse.ArgumentParser") -> None:
        """Add arguments specific to the export-login command."""
        parser.add_argument(
            "--base",
            metavar="base",
            type=str,
            help="Show plugins for <base>",
        )

    @overrides
    def run(self, parsed_args):
        if parsed_args.base in ("core18", "core20"):
            raise errors.LegacyFallback()

        base = parsed_args.base
        message: Optional[str] = None

        if base is None:
            try:
                snap_project = get_snap_project()
                # Run this to trigger legacy behavior
                yaml_data = process_yaml(snap_project.project_file)
                project = Project.unmarshal(yaml_data)
                base = project.get_effective_base()
                message = (
                    f"Displaying plugins available to the current base {base!r} project"
                )
            except errors.ProjectMissing:
                base = "core22"

        if message is None:
            message = f"Displaying plugins available for {base!r}"

        if base != "core22":
            raise errors.SnapcraftError(f"{base} not supported")

        registered_plugins = get_registered_plugins()

        emit.message(message + "\n" + "\n".join(n for n in registered_plugins))


class PluginsCommand(ListPluginsCommand, abc.ABC):
    """A command alias to list the available plugins."""

    name = "plugins"
    hidden = True
