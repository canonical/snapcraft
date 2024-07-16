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

"""Snapcraft Package service."""

from __future__ import annotations

import os
import pathlib
import shutil
from typing import TYPE_CHECKING, cast

from craft_application import AppMetadata, PackageService
from craft_application.models import BuildInfo
from craft_application.util import strtobool
from overrides import override

from snapcraft import errors, linters, models, pack
from snapcraft.linters import LinterStatus
from snapcraft.meta import component_yaml, snap_yaml
from snapcraft.parts import extract_metadata as extract
from snapcraft.parts import update_metadata as update
from snapcraft.parts.setup_assets import setup_assets
from snapcraft.services import Lifecycle
from snapcraft.utils import process_version

if TYPE_CHECKING:
    from snapcraft.services import SnapcraftServiceFactory


class Package(PackageService):
    """Package service subclass for Snapcraft."""

    _project: models.Project

    def __init__(  # noqa: PLR0913 (Too many arguments)
        self,
        app: AppMetadata,
        services: SnapcraftServiceFactory,
        *,
        project: models.Project,
        snapcraft_yaml_path: pathlib.Path,
        build_plan: list[BuildInfo],
        parse_info: dict[str, list[str]],
    ) -> None:
        super().__init__(app, services, project=project)
        self._snapcraft_yaml_path = snapcraft_yaml_path
        self._build_plan = build_plan
        self._platform = build_plan[0].platform
        self._build_for = build_plan[0].build_for
        self._parse_info = parse_info

    @override
    def _extra_project_updates(self) -> None:
        # Update the project from parse-info data.
        project_info = self._services.lifecycle.project_info
        extracted_metadata = extract.extract_lifecycle_metadata(
            self._project.adopt_info,
            self._parse_info,
            project_info.work_dir,
            partitions=project_info.partitions,
        )
        update.update_from_extracted_metadata(
            self._project,
            metadata_list=extracted_metadata,
            assets_dir=self._get_assets_dir(),
            prime_dir=project_info.prime_dir,
        )

    def _pack_components(self, dest: pathlib.Path) -> list[pathlib.Path]:
        component_files: list[pathlib.Path] = []
        for component in self._project.get_component_names():
            filename = pack.pack_component(
                cast(Lifecycle, self._services.lifecycle).get_prime_dir(component),
                compression=self._project.compression,
                output_dir=dest,
            )
            component_files.append(pathlib.Path(filename))

        return component_files

    @override
    def pack(self, prime_dir: pathlib.Path, dest: pathlib.Path) -> list[pathlib.Path]:
        """Create one or more packages as appropriate.

        :param prime_dir: Path to the directory to pack.
        :param dest: Directory into which to write the package(s).
        :returns: A list of paths to created packages.
        """
        issues = linters.run_linters(prime_dir, lint=self._project.lint)
        status = linters.report(issues, intermediate=True)

        # In case of linter errors, stop execution and return the error code.
        if status in (LinterStatus.ERRORS, LinterStatus.FATAL):
            raise errors.LinterError("Linter errors found", exit_code=status)

        snap_file = pathlib.Path(
            pack.pack_snap(
                prime_dir,
                output=str(dest),
                compression=self._project.compression,
                name=self._project.name,
                version=process_version(self._project.version),
                target_arch=self._build_plan[0].build_for,
            )
        )
        component_files = self._pack_components(dest)

        return [snap_file] + component_files

    def _get_assets_dir(self) -> pathlib.Path:
        """Return a snapcraft assets directory.

        Asset directories can exist in:

        - <PROJECT_ROOT>/snap
        - <PROJECT_ROOT>/build-aux/snap
        """
        project_dir = self._services.lifecycle.project_info.project_dir
        for asset_reldir in ("snap", "build-aux/snap"):
            asset_dir = project_dir / asset_reldir
            if asset_dir.exists():
                return asset_dir

        # This is for backwards compatibility with setup_assets(...)
        return project_dir / "snap"

    @override
    def write_metadata(self, path: pathlib.Path) -> None:
        """Write the project metadata to metadata.yaml in the given directory.

        :param path: The path to the prime directory.
        """
        meta_dir = path / "meta"
        meta_dir.mkdir(parents=True, exist_ok=True)
        self.metadata.to_yaml_file(meta_dir / "snap.yaml")

        enable_manifest = strtobool(os.getenv("SNAPCRAFT_BUILD_INFO", "n"))

        # Snapcraft's Lifecycle implementation is what we need to refer to for typing
        lifecycle_service = cast(Lifecycle, self._services.lifecycle)
        if enable_manifest:
            snap_dir = path / "snap"
            snap_dir.mkdir(parents=True, exist_ok=True)
            manifest = lifecycle_service.generate_manifest()
            manifest.to_yaml_file(snap_dir / "manifest.yaml")

            shutil.copy(self._snapcraft_yaml_path, snap_dir)

        assets_dir = self._get_assets_dir()
        setup_assets(
            self._project,
            assets_dir=assets_dir,
            project_dir=self._services.lifecycle.project_info.project_dir,
            prime_dirs=lifecycle_service.prime_dirs,
            meta_directory_handler=meta_directory_handler,
        )

        for component in self._project.get_component_names():
            component_yaml.write(
                project=self._project,
                component_name=component,
                component_prime_dir=lifecycle_service.get_prime_dir(component),
            )

    @property
    def metadata(self) -> snap_yaml.SnapMetadata:
        """Get the metadata model for this project."""
        return snap_yaml.get_metadata_from_project(
            self._project,
            self._services.lifecycle.prime_dir,
            arch=self._build_plan[0].build_for,
        )


def _hardlink_or_copy(source: pathlib.Path, destination: pathlib.Path) -> bool:
    """Try to hardlink and fallback to copy if it fails.

    :param source: the source path.
    :param destination: the destination path.
    :returns: True if a hardlink was done or False for copy.
    """
    # Unlink the destination to avoid link failures
    destination.unlink(missing_ok=True)

    try:
        destination.hardlink_to(source)
    except OSError as os_error:
        # Cross device link
        if os_error.errno != 18:
            raise
        shutil.copy(source, destination)
        return False

    return True


def meta_directory_handler(assets_dir: pathlib.Path, path: pathlib.Path):
    """Handle hooks and gui assets from Snapcraft.

    :param assets_dir: directory with project assets.
    :param path: directory to write assets to.
    """
    meta_dir = path / "meta"
    built_snap_hooks = path / "snap" / "hooks"
    hooks_project_dir = assets_dir / "hooks"

    hooks_meta_dir = meta_dir / "hooks"

    if built_snap_hooks.is_dir():
        hooks_meta_dir.mkdir(parents=True, exist_ok=True)
        for hook in built_snap_hooks.iterdir():
            meta_dir_hook = hooks_meta_dir / hook.name
            # Remove to always refresh to the latest
            meta_dir_hook.unlink(missing_ok=True)
            meta_dir_hook.hardlink_to(hook)

    # Overwrite any built hooks with project level ones
    if hooks_project_dir.is_dir():
        hooks_meta_dir.mkdir(parents=True, exist_ok=True)
        for hook in hooks_project_dir.iterdir():
            meta_dir_hook = hooks_meta_dir / hook.name

            _hardlink_or_copy(hook, meta_dir_hook)

    # Write any gui assets
    gui_project_dir = assets_dir / "gui"
    gui_meta_dir = meta_dir / "gui"
    if gui_project_dir.is_dir():
        gui_meta_dir.mkdir(parents=True, exist_ok=True)
        for gui in gui_project_dir.iterdir():
            meta_dir_gui = gui_meta_dir / gui.name

            _hardlink_or_copy(gui, meta_dir_gui)
