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

"""External metadata helpers."""

from pathlib import Path
from typing import Dict, Final, List

import pydantic
from craft_cli import emit

from snapcraft import errors
from snapcraft.meta import ExtractedMetadata
from snapcraft.projects import MANDATORY_ADOPTABLE_FIELDS, Project

_VALID_ICON_EXTENSIONS: Final[List[str]] = ["png", "svg"]


def update_project_metadata(
    project: Project,
    *,
    project_vars: Dict[str, str],
    metadata_list: List[ExtractedMetadata],
    assets_dir: Path,
    prime_dir: Path,
) -> None:
    """Set project fields using corresponding adopted entries.

    Fields are validated on assignment by pydantic.

    :param project: The project to update.
    :param project_vars: The variables updated during lifecycle execution.
    :param metadata_list: List containing parsed information from metadata files.

    :raises SnapcraftError: If project update failed.
    """
    _update_project_variables(project, project_vars)

    for metadata in metadata_list:
        # Data specified in the project yaml has precedence over extracted data
        if metadata.title and not project.title:
            project.title = metadata.title

        if metadata.summary and not project.summary:
            project.summary = metadata.summary

        if metadata.description and not project.description:
            project.description = metadata.description

        if metadata.version and not project.version:
            project.version = metadata.version

        if metadata.grade and not project.grade:
            project.grade = metadata.grade  # type: ignore

        emit.debug(f"project icon: {project.icon!r}")
        emit.debug(f"metadata icon: {metadata.icon!r}")

        if not project.icon:
            _update_project_icon(project, metadata=metadata, assets_dir=assets_dir)

        _update_project_app_desktop_file(
            project, metadata=metadata, assets_dir=assets_dir, prime_dir=prime_dir
        )

    # Fields that must not end empty
    for field in MANDATORY_ADOPTABLE_FIELDS:
        if not getattr(project, field):
            raise errors.SnapcraftError(
                f"Field {field!r} was not adopted from metadata"
            )


def _update_project_variables(project: Project, project_vars: Dict[str, str]):
    """Update project fields with values set during lifecycle processing."""
    try:
        if project_vars["version"]:
            project.version = project_vars["version"]
        if project_vars["grade"]:
            project.grade = project_vars["grade"]  # type: ignore
    except pydantic.ValidationError as err:
        _raise_formatted_validation_error(err)
        raise errors.SnapcraftError(f"error setting variable: {err}")


def _update_project_icon(
    project: Project,
    *,
    metadata: ExtractedMetadata,
    assets_dir: Path,
) -> None:
    """Look for icons files and update project.

    Existing icon in snap/gui/icon.{png,svg} has precedence over extracted data
    """
    icon_files = (f"{assets_dir}/gui/icon.{ext}" for ext in _VALID_ICON_EXTENSIONS)

    for icon_file in icon_files:
        if Path(icon_file).is_file():
            break
    else:
        if metadata.icon:
            project.icon = metadata.icon

    emit.debug(f"updated project icon: {project.icon}")


def _update_project_app_desktop_file(
    project: Project, *, metadata: ExtractedMetadata, assets_dir: Path, prime_dir: Path
) -> None:
    """Look for desktop files and update project.

    Existing desktop file snap/gui/<appname>.desktop has precedence over extracted data
    """
    if metadata.common_id and project.apps:
        app_name = None
        for name, data in project.apps.items():
            if data.common_id == metadata.common_id:
                app_name = name
                break

        if not app_name:
            emit.debug(f"no app declares id {metadata.common_id!r}")
            return

        if project.apps[app_name].desktop:
            emit.debug(f"app {app_name!r} already declares a desktop file")
            return

        emit.debug(
            f"look for desktop file with id {metadata.common_id!r} in app {app_name!r}"
        )

        desktop_file = f"{assets_dir}/gui/{app_name}.desktop"
        if Path(desktop_file).is_file():
            emit.debug(f"use already existing desktop file {desktop_file!r}")
            return

        if metadata.desktop_file_paths:
            for filename in metadata.desktop_file_paths:
                if Path(prime_dir, filename.lstrip("/")).is_file():
                    project.apps[app_name].desktop = filename
                    emit.debug(f"use desktop file {filename!r}")
                    break


def _raise_formatted_validation_error(err: pydantic.ValidationError):
    error_list = err.errors()
    if len(error_list) != 1:
        return

    error = error_list[0]
    loc = error.get("loc")
    msg = error.get("msg")

    if not (loc and msg) or not isinstance(loc, tuple):
        return

    varname = ".".join((x for x in loc if isinstance(x, str)))
    raise errors.SnapcraftError(f"error setting {varname}: {msg}")
