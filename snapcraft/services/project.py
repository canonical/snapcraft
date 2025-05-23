# Copyright 2025 Canonical Ltd.
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

"""Snapcraft Project service."""

from __future__ import annotations

import pathlib
from typing import TYPE_CHECKING, Any, cast

import craft_cli
import craft_platforms
import craft_providers.bases
from craft_application import ProjectService
from overrides import override

from snapcraft.extensions import apply_extensions
from snapcraft.models.project import ComponentProject, Platform, apply_root_packages
from snapcraft.parts import set_global_environment
from snapcraft.parts.yaml_utils import extract_parse_info, get_snap_project
from snapcraft.providers import SNAPCRAFT_BASE_TO_PROVIDER_BASE
from snapcraft.utils import get_effective_base

if TYPE_CHECKING:
    import craft_parts


class Project(ProjectService):
    """Snapcraft-specific project service."""

    __project_file_path: pathlib.Path | None = None

    @staticmethod
    @override
    def _app_preprocess_project(
        project: dict[str, Any],
        *,
        build_on: str,
        build_for: str,
        platform: str,
    ) -> None:
        apply_extensions(project, arch=build_on, target_arch=build_for)
        # craft-parts doesn't allow `parse-info` in parts
        extract_parse_info(project)
        apply_root_packages(project)

    def get_parse_info(self) -> dict[str, list[str]]:
        return extract_parse_info(self.get_raw())

    @override
    def resolve_project_file_path(self) -> pathlib.Path:
        """Overridden to handle all locations for a snapcraft.yaml (craft-application#583)"""
        if self.__project_file_path:
            return self.__project_file_path

        self.__project_file_path = get_snap_project(
            project_dir=self._project_dir
        ).project_file
        craft_cli.emit.debug(
            f"Project file found at {str(self.__project_file_path.resolve())!r}"
        )
        return self.__project_file_path

    @override
    def get_partitions_for(
        self,
        *,
        platform: str,
        build_for: str,
        build_on: craft_platforms.DebianArchitecture,
    ) -> list[str] | None:
        project = self._preprocess(
            build_for=build_for, build_on=cast(str, build_on), platform=platform
        )
        components = ComponentProject.unmarshal(project)
        return components.get_partitions()

    @override
    def _app_render_legacy_platforms(self) -> dict[str, craft_platforms.PlatformDict]:
        project = self.get_raw()
        effective_base = SNAPCRAFT_BASE_TO_PROVIDER_BASE.get(
            str(
                get_effective_base(
                    base=project.get("base"),
                    build_base=project.get("build-base"),
                    project_type=project.get("type"),
                    name=project.get("name"),
                    translate_devel=False,
                )
            )
        )

        # For backwards compatibility with core22, convert the platforms.
        if (
            effective_base == craft_providers.bases.BuilddBaseAlias.JAMMY
            and project.get("architectures")
        ):
            platforms: dict[str, craft_platforms.PlatformDict] = {
                key: cast(craft_platforms.PlatformDict, value.marshal())
                for key, value in Platform.from_architectures(
                    project["architectures"]
                ).items()
            }
        else:
            host_arch = str(craft_platforms.DebianArchitecture.from_host())
            platforms = {
                host_arch: {
                    "build-on": [host_arch],
                    "build-for": [host_arch],
                }
            }

        return platforms

    @override
    def update_project_environment(self, info: craft_parts.ProjectInfo) -> None:
        """Set global environment variables."""
        super().update_project_environment(info)
        set_global_environment(info)
