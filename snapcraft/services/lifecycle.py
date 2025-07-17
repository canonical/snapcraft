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
import json
import os
from datetime import datetime
from pathlib import Path
from typing import Any, cast

from craft_application import AppMetadata, LifecycleService, ServiceFactory
from craft_parts import StepInfo, callbacks
from craft_parts.packages import Repository as Repo
from overrides import overrides

from snapcraft import __version__, errors, models, os_release, parts, utils
from snapcraft.parts.yaml_utils import get_snap_project


class Lifecycle(LifecycleService):
    """Snapcraft specialization of the Lifecycle Service."""

    def __init__(  # noqa: PLR0913 (too many arguments)
        self,
        app: AppMetadata,
        services: ServiceFactory,
        *,
        work_dir: Path | str,
        cache_dir: Path | str,
        **lifecycle_kwargs: Any,  # noqa: ANN401 - eventually used in an Any
    ) -> None:
        super().__init__(
            app,
            services,
            work_dir=work_dir,
            cache_dir=cache_dir,
            **lifecycle_kwargs,
        )
        self._start_time = datetime.now()
        self._manifest: models.Manifest

    @overrides
    def setup(self) -> None:
        project = cast(models.Project, self._services.get("project").get())

        if project.package_repositories:
            # Note: we unfortunately need to handle missing gpg/dirmngr binaries
            # ourselves here, as this situation happens in Launchpad (where
            # builds are executed destructively).
            required_packages = ["gpg", "dirmngr"]
            if any(p for p in required_packages if not Repo.is_package_installed(p)):
                Repo.install_packages(required_packages, refresh_package_cache=False)

        # Have the lifecycle install the base snap, and look into it when
        # determining the package cutoff.
        self._manager_kwargs.update(
            base=project.get_effective_base(),
            extra_build_snaps=project.get_extra_build_snaps(),
            confinement=project.confinement,
            project_base=project.base or "",
            project_name=project.name,
        )
        callbacks.register_prologue(parts.set_global_environment)
        callbacks.register_pre_step(parts.set_step_environment)
        super().setup()

    @overrides
    def post_prime(self, step_info: StepInfo) -> bool:
        """Run post-prime parts steps for Snapcraft."""
        from snapcraft.parts.plugins import (  # noqa: PLC0415 (import-outside-top-level)
            python_common,
        )

        project = cast(models.Project, self._project)

        part_name = step_info.part_name
        plugin_name = project.parts[part_name]["plugin"]

        # Handle plugin-specific prime fixes
        if plugin_name in python_common.get_python_plugins().keys():
            python_common.post_prime(step_info)

        # Handle patch-elf

        # do not use system libraries in classic confinement
        use_system_libs = not bool(project.confinement == "classic")

        return parts.patch_elf(step_info, use_system_libs=use_system_libs)

    def get_prime_dir(self, component: str | None = None) -> Path:
        """Get the prime directory path for the default prime dir or a component.

        :param component: Name of the component to get the prime directory for.

        :returns: The default prime directory or a component's prime directory.

        :raises SnapcraftError: If the component does not exist.
        """
        try:
            return self.prime_dirs[component]
        except KeyError as err:
            raise errors.SnapcraftError(
                f"Could not get prime directory for component {component!r} "
                "because it does not exist."
            ) from err

    @property
    def prime_dirs(self) -> dict[str | None, Path]:
        """Return a mapping of component names to prime directories.

        'None' maps to the default prime directory.
        """
        return utils.get_prime_dirs_from_project(self._lcm.project_info)

    def generate_manifest(self) -> models.Manifest:
        """Create and populate the manifest file."""
        primed_stage_packages: set[str] = set()

        image_information = os.getenv("SNAPCRAFT_IMAGE_INFO", "{}")
        try:
            image_info = json.loads(image_information)
        except json.decoder.JSONDecodeError as err:
            raise errors.SnapcraftError(
                f"Image information decode error at {err.lineno}:{err.colno}: "
                f"{err.doc!r}: {err.msg}"
            ) from err

        project = cast(models.Project, self._project)

        project_parts = copy.deepcopy(project.parts)
        for name, part in project_parts.items():
            assets = self.get_pull_assets(part_name=name)
            if assets:
                part["stage-packages"] = assets.get("stage-packages", []) or []
            for key in ("stage", "prime", "stage-packages", "build-packages"):
                part.setdefault(key, [])

            stage_packages = self.get_primed_stage_packages(part_name=name)
            if stage_packages:
                primed_stage_packages |= set(stage_packages)

        osrel = os_release.OsRelease()
        version = utils.process_version(project.version)
        build_for = self._services.get("build_plan").plan()[0].build_for

        return models.Manifest(
            # Snapcraft annotations
            snapcraft_version=__version__,
            snapcraft_started_at=self._start_time.isoformat("T") + "Z",
            snapcraft_os_release_id=osrel.name().lower(),
            snapcraft_os_release_version_id=osrel.version_id().lower(),
            # Project fields
            name=project.name,
            version=version,
            summary=str(project.summary),
            description=str(project.description),
            base=project.base,
            grade=project.grade or "stable",
            confinement=project.confinement,
            apps=project.apps,
            parts=project_parts,
            # Architecture
            architectures=[build_for],
            # Image info
            image_info=image_info,
            # Build environment
            build_packages=[],
            build_snaps=[],
            primed_stage_packages=sorted(primed_stage_packages),
        )

    @overrides
    def _get_local_keys_path(self) -> Path | None:
        snap_project = get_snap_project()
        keys_dir = snap_project.assets_dir / "keys"
        if keys_dir.is_dir():
            return keys_dir

        return None
