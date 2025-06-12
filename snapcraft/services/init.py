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

"""Service for initializing a project."""

import pathlib

import craft_application.errors
import craft_cli
from craft_application import services
from typing_extensions import override

from snapcraft import errors
from snapcraft.models.project import validate_name
from snapcraft.parts.yaml_utils import get_snap_project


class Init(services.InitService):
    """Service class for initializing a project."""

    @override
    def initialise_project(
        self,
        *,
        project_dir: pathlib.Path,
        project_name: str,
        template_dir: pathlib.Path,
    ) -> None:
        try:
            validate_name(name=project_name, field_name="snap")
            if len(project_name) > 40:
                raise ValueError("snap names must be 40 characters or less")
        except ValueError as err:
            raise errors.SnapcraftError(
                message=f"Invalid snap name {project_name!r}: {str(err)}.",
                resolution="Provide a valid name with '--name' or rename the project directory.",
            ) from err

        super().initialise_project(
            project_dir=project_dir,
            project_name=project_name,
            template_dir=template_dir,
        )
        craft_cli.emit.message(
            "See https://documentation.ubuntu.com/snapcraft/stable/reference/"
            "project-file for reference information about the snapcraft.yaml format."
        )

    @override
    def check_for_existing_files(
        self,
        *,
        project_dir: pathlib.Path,
        template_dir: pathlib.Path,
    ) -> None:
        init_profile = template_dir.name
        if init_profile != "test":
            try:
                craft_cli.emit.progress("Checking for an existing 'snapcraft.yaml'.")
                project = get_snap_project(project_dir)
            # the `ProjectMissing` error means a new project can be initialised
            except craft_application.errors.ProjectDirectoryTypeError:
                raise errors.SnapcraftError(
                    "Could not initialise a new snapcraft project because "
                    "a file named 'snap' already exists.",
                    details="The 'snap' name is reserved for the project directory.",
                    resolution="Rename or remove the file named 'snap'.",
                )
            except craft_application.errors.ProjectFileError:
                craft_cli.emit.debug("Could not find an existing 'snapcraft.yaml'.")
            else:
                raise errors.SnapcraftError(
                    "Could not initialise a new snapcraft project because "
                    f"{str(project.project_file)!r} already exists"
                )

        super().check_for_existing_files(
            project_dir=project_dir,
            template_dir=template_dir,
        )
