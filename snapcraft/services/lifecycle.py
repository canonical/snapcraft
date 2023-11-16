# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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
"""Snapcraft Lifecycle service."""

from __future__ import annotations

import pathlib
from typing import Any, List, Set

import craft_parts
from craft_application import LifecycleService, ServiceFactory
from craft_application.application import ReadOnlyAppConfig, AppMetadata
from craft_archives import repo
from craft_cli import emit
from craft_parts import Step, callbacks
from overrides import override

from snapcraft import models, application, ua_manager
from snapcraft.meta import ExtractedMetadata, snap_yaml
from snapcraft.parts import lifecycle, plugins, parts
from snapcraft.parts.lifecycle import generate_manifest
from snapcraft.parts.setup_assets import setup_assets
from snapcraft.parts.update_metadata import update_project_metadata


class Lifecycle(LifecycleService):
    """Snapcraft-specific lifecycle service."""

    _project: models.Project
    _config: application.SnapcraftConfig  # type: ignore
    __is_setup = False

    def __init__(  # noqa: PLR0913 (too many arguments)
        self,
        app: AppMetadata,
        services: ServiceFactory,
        *,
        project: models.Project,
        work_dir: pathlib.Path | str,
        cache_dir: pathlib.Path | str,
        build_for: str,
        config: ReadOnlyAppConfig | None = None,
        **lifecycle_kwargs: Any,  # noqa: ANN401 - eventually used in an Any
    ) -> None:
        self._enable_manifest = lifecycle_kwargs.pop("enable_manifest", False)
        self._manifest_image_info = "{}"
        super().__init__(
            app,
            services,
            project=project,
            work_dir=work_dir,
            cache_dir=cache_dir,
            build_for=build_for,
            config=config,
            **lifecycle_kwargs
        )

    @override
    def setup(self) -> None:
        """Initialize the LifecycleManager with previously-set arguments."""
        plugins.register()

        callbacks.register_prologue(lifecycle.set_global_environment)
        callbacks.register_pre_step(lifecycle.set_step_environment)
        callbacks.register_post_step(lifecycle.patch_elf, step_list=[Step.PRIME])

        # Configure extra args to the LifecycleManager
        project_vars = {
            "version": self._project.version or "",
            "grade": self._project.grade or "",
        }

        self._manager_kwargs.update(
            assets_dir=self._config.assets_dir,
            package_repositories=self._project.package_repositories or [],
            project_name=self._project.name,
            project_vars_part_name=self._project.adopt_info,
            project_vars=project_vars,
            project_base=self._project.base,
            confinement=self._project.confinement,
            extra_build_snaps=self._project.get_extra_build_snaps(),
        )

        self.__is_setup = True

        super().setup()

    def enable_manifest(self, image_info: str | None) -> None:
        """Enable writing of the snap manifest."""
        if self.__is_setup:
            raise RuntimeError("Too late to enable tracking of stage packages.")
        self._manager_kwargs.update(track_stage_packages=True)
        self._enable_manifest = True
        if image_info:
            self._manifest_image_info = image_info

    def _add_repositories(self) -> None:
        """Install package repositories in the environment."""
        if not self._project.package_repositories:
            return

        emit.progress(f"Installing {len(self._project.package_repositories)} repositories...")
        refresh_required = repo.install(
            self._project.package_repositories, key_assets=self._config.assets_dir / "keys"
        )
        if refresh_required:
            emit.progress("Refreshing package repositories...")
            # TODO: craft-parts API for: force_refresh=refresh_required
            # pylint: disable=C0415
            from craft_parts.packages import deb

            deb.Ubuntu.refresh_packages_list.cache_clear()
            self._lcm.refresh_packages_list()
        emit.progress("Installed package repositories", permanent=True)

    @override
    def run(self, step_name: str | None, part_names: list[str] | None = None) -> None:
        with ua_manager.ua_manager(self._config.ua_token, services=self._project.ua_services):
            self._add_repositories()
            # TODO: Force re-running prime when running snapcraft try.
            super().run(step_name, part_names)

    def post_prime(self) -> None:
        emit.progress("Extracting and updating metadata...")
        update_project_metadata(
            self._project,
            project_vars={
                "version": self._lcm.project_info.get_project_var("version"),
                "grade": self._lcm.project_info.get_project_var("grade"),
            },
            metadata_list=self._get_metadata(),
            assets_dir=self._config.assets_dir,
            prime_dir=self.prime_dir,
        )

        emit.progress("Copying snap assets...")
        setup_assets(
            self._project,
            assets_dir=self._config.assets_dir,
            project_dir=self._config.project_dir,
            prime_dir=self.prime_dir,
        )

        emit.progress("Generating snap metadata...")
        snap_yaml.write(
            self._project, self.prime_dir, arch=self._project.get_build_for()
        )
        emit.progress("Generated snap metadata", permanent=True)

        if self._enable_manifest:
            emit.progress("Writing manifest")
            generate_manifest(
                self._project,
                start_time=self._config.start_time,
                image_information=self._manifest_image_info,
                get_pull_assets=self._lcm.get_pull_assets,
                prime_dir=self.prime_dir,
                primed_stage_packages=self.get_primed_stage_packages()
            )
            emit.progress("Snap manifest written.", permanent=True)

    def get_primed_stage_packages(self) -> List[str]:
        """Obtain the list of primed stage packages from all parts."""
        primed_stage_packages: Set[str] = set()
        for name in self._project.parts:
            stage_packages = self._lcm.get_primed_stage_packages(part_name=name)
            if stage_packages:
                primed_stage_packages |= set(stage_packages)
        package_list = list(primed_stage_packages)
        package_list.sort()
        return package_list

    def _get_metadata(self) -> list[ExtractedMetadata]:
        if not self._project.adopt_info or self._project.adopt_info not in self._config.parse_info:
            return []
        part = craft_parts.Part(self._project.adopt_info, {}, project_dirs=self.project_info.dirs)

        return parts.get_metadata_from_part(part, self._config.parse_info[self._project.adopt_info])
