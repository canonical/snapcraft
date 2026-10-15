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

from typing import TYPE_CHECKING, Any, cast

import craft_cli
import craft_platforms
import craft_providers.bases
from craft_application import ProjectService
from craft_application.errors import CraftValidationError
from craft_application.models.constraints import LicenseStr
from craft_application.util import humanize_list, is_managed_mode
from pydantic import TypeAdapter, ValidationError
from typing_extensions import override

from snapcraft.extensions import apply_extensions
from snapcraft.models.project import ComponentProject, Platform, apply_root_packages
from snapcraft.parts import set_global_environment
from snapcraft.parts.yaml_utils import (
    create_project_vars,
    extract_parse_info,
    get_snap_project,
)
from snapcraft.providers import SNAPCRAFT_BASE_TO_PROVIDER_BASE
from snapcraft.utils import get_effective_base

if TYPE_CHECKING:
    import pathlib

    import craft_parts

_SNAP_DIRECTORY_NAME = "snap"


class Project(ProjectService):
    """Snapcraft-specific project service."""

    __project_file_path: pathlib.Path | None = None

    # Used to only issue key warnings once.
    _ua_service_warning: bool = False
    _license_spdx_warning: bool = False
    _snap_dir_source_warning: bool = False

    @override
    def _app_preprocess_project(
        self,
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
        Project.validate_ua_services(project)
        Project.validate_license_spdx(project)
        self._warn_if_sources_resolve_from_parent(project)

    def _warn_if_sources_resolve_from_parent(self, project: dict[str, Any]) -> None:
        """Warn when part sources only resolve from the parent of a 'snap' directory.

        A project may keep its 'snapcraft.yaml' inside a 'snap' subdirectory while
        writing part sources relative to the repository root. Running snapcraft from
        within 'snap' resolves those sources against the wrong directory, and
        craft-parts reports an obscure "unable to determine source type" error instead
        of saying the source was not found.
        """
        if self._snap_dir_source_warning:
            return
        self._snap_dir_source_warning = True

        project_dir = self._project_dir
        if project_dir.name != _SNAP_DIRECTORY_NAME:
            return

        # Only warn when the project file sits directly in the project directory.
        # Nested layouts such as 'build-aux/snap/snapcraft.yaml' are unambiguous.
        project_file = self.resolve_project_file_path()
        if (project_dir / project_file).parent != project_dir:
            return

        missing = self._find_sources_resolving_from_parent(project, project_dir)
        if not missing:
            return

        craft_cli.emit.warning(
            f"Part source {humanize_list(missing, 'and')} does not exist relative to "
            f"{str(project_dir)!r}, but does exist relative to its parent. "
            f"Snapcraft may be running from the wrong directory - try running it "
            f"from {str(project_dir.parent)!r} instead."
        )

    @staticmethod
    def _find_sources_resolving_from_parent(
        project: dict[str, Any], project_dir: pathlib.Path
    ) -> list[str]:
        """Return sources that exist in the parent of project_dir but not in it.

        URLs and absolute paths resolve identically from both directories, so they
        never appear here; that keeps the check specific to local sources without
        having to reimplement craft-parts' source type detection.
        """
        parent_dir = project_dir.parent
        missing = []
        for part in (project.get("parts") or {}).values():
            if not isinstance(part, dict):
                continue
            source = part.get("source")
            if not isinstance(source, str) or not source:
                continue
            if (project_dir / source).exists():
                continue
            if (parent_dir / source).exists():
                missing.append(source)
        return missing

    def get_parse_info(self) -> dict[str, list[str]]:
        return extract_parse_info(self.get_raw())

    @classmethod
    def validate_license_spdx(cls, project: dict[str, Any]) -> None:
        if cls._license_spdx_warning:
            return
        cls._license_spdx_warning = True

        if is_managed_mode():
            return

        lic = project.get("license")
        if lic is None:
            return

        try:
            TypeAdapter(LicenseStr).validate_python(lic)
        except ValidationError:
            craft_cli.emit.warning(
                "Non-SPDX licenses are deprecated. Use SPDX license strings or 'proprietary' instead. For more information, see https://spdx.org/licenses/."
            )

    @classmethod
    def validate_ua_services(cls, project: dict[str, Any]) -> None:
        """Warn if the 'ua-services' key is used for a base other than core22.

        This warning is shown no more than once - either before launching a managed
        instance or in destructive mode.
        """
        # one-shot
        if cls._ua_service_warning:
            return
        cls._ua_service_warning = True

        if is_managed_mode():
            return

        base = get_effective_base(
            base=project.get("base"),
            build_base=project.get("build-base"),
            project_type=project.get("type"),
            name=project.get("name"),
            translate_devel=True,
        )
        if base is not None and base != "core22" and project.get("ua-services"):
            craft_cli.emit.warning(
                f"The 'ua-services' key is ignored for {base!r}. "
                "Use '--pro=<services>' instead."
            )

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
        if project.get("base") == "core22" and "platforms" in project:
            raise CraftValidationError(
                "'platforms' key is not supported for base 'core22'.",
                resolution="Use 'architectures' key instead.",
            )

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

        platforms: dict[str, craft_platforms.PlatformDict]
        # For backwards compatibility with core22, convert the platforms.
        if (
            effective_base == craft_providers.bases.BuilddBaseAlias.JAMMY
            and project.get("architectures")
        ):
            platforms = {
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

    @override
    def _create_project_vars(
        self, project: dict[str, Any]
    ) -> craft_parts.ProjectVarInfo:
        return create_project_vars(project)
