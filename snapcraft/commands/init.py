# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023-2024 Canonical Ltd.
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

"""Snapcraft init command."""

import argparse

import craft_application.commands
import craft_cli
from typing_extensions import override

from snapcraft import errors
from snapcraft.parts.yaml_utils import get_snap_project


class InitCommand(craft_application.commands.InitCommand):
    """Snapcraft init command."""

    @override
    def run(self, parsed_args: argparse.Namespace) -> None:
        """Pack a directory or run the lifecycle and pack all artifacts."""
        project_dir = self._get_project_dir(parsed_args)

        if parsed_args.name:
            craft_cli.emit.progress(
                "Ignoring '--name' parameter because it is not supported yet.",
                permanent=True,
            )

        try:
            craft_cli.emit.progress("Checking for an existing 'snapcraft.yaml'.")
            project = get_snap_project(project_dir)
            raise errors.SnapcraftError(
                "could not initialise a new snapcraft project because "
                f"{str(project.project_file)!r} already exists"
            )
            # the `ProjectMissing` error means a new project can be initialized
        except errors.ProjectMissing:
            craft_cli.emit.progress("Could not find an existing 'snapcraft.yaml'.")

        super().run(parsed_args)

        craft_cli.emit.message(
            "Go to https://docs.snapcraft.io/the-snapcraft-format/8337 for more "
            "information about the snapcraft.yaml format."
        )
