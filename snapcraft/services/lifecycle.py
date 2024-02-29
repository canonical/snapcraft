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
"""Snapcraft Lifecycle Service."""

import copy
import os
import shutil
from datetime import datetime
from pathlib import Path
from typing import Any, cast

from craft_application import AppMetadata, LifecycleService, ServiceFactory
from craft_application.models import BuildInfo
from craft_cli import emit
from overrides import overrides

from snapcraft import models, utils
from snapcraft.meta import manifest


class Lifecycle(LifecycleService):
    """Snapcraft specialization of the Lifecycle Service."""

    def __init__(  # noqa: PLR0913 (too many arguments)
        self,
        app: AppMetadata,
        services: ServiceFactory,
        *,
        project: models.Project,
        work_dir: Path | str,
        cache_dir: Path | str,
        build_plan: list[BuildInfo],
        project_path: Path | None,
        **lifecycle_kwargs: Any,  # noqa: ANN401 - eventually used in an Any
    ) -> None:
        super().__init__(
            app,
            services,
            project=project,
            work_dir=work_dir,
            cache_dir=cache_dir,
            build_plan=build_plan,
            **lifecycle_kwargs,
        )
        self._project_path = project_path
        self._start_time = datetime.now()

    @overrides
    def setup(self) -> None:
        project = cast(models.Project, self._project)

        # Have the lifecycle install the base snap, and look into it when
        # determining the package cutoff.
        self._manager_kwargs.update(
            base=project.get_effective_base(),
            extra_build_snaps=project.get_extra_build_snaps(),
        )
        super().setup()

    @overrides
    def run(self, step_name: str | None, part_names: list[str] | None = None) -> None:
        super().run(step_name, part_names)

        enable_manifest = utils.strtobool(os.getenv("SNAPCRAFT_BUILD_INFO", "n"))
        if enable_manifest:
            self._generate_manifest()

    def _generate_manifest(self) -> None:
        """Create and populate the manifest file."""
        emit.progress("Generating snap manifest...")
        image_information = os.getenv("SNAPCRAFT_IMAGE_INFO", "{}")
        primed_stage_packages: set[str] = set()

        project = cast(models.Project, self._project)

        parts = copy.deepcopy(project.parts)
        for name, part in parts.items():
            assets = self.get_pull_assets(part_name=name)
            if assets:
                part["stage-packages"] = assets.get("stage-packages", []) or []
            for key in ("stage", "prime", "stage-packages", "build-packages"):
                part.setdefault(key, [])

            stage_packages = self.get_primed_stage_packages(part_name=name)
            if stage_packages:
                primed_stage_packages |= set(stage_packages)

        host_arch = utils.get_host_architecture()
        build_for = self._build_plan[0].build_for if self._build_plan else host_arch

        manifest.write(
            project,
            self.prime_dir,
            arch=build_for,
            parts=parts,
            start_time=self._start_time,
            image_information=image_information,
            primed_stage_packages=list(primed_stage_packages),
        )
        emit.progress("Generated snap manifest")

        # Also copy the original snapcraft.yaml
        if self._project_path:
            shutil.copy(self._project_path, self.prime_dir / "snap")
