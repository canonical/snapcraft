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

"""Snapcraft Package service."""

from __future__ import annotations

import pathlib
from typing import TYPE_CHECKING

from craft_application import AppMetadata, PackageService
from overrides import override

from snapcraft import errors, linters, models, pack
from snapcraft.linters import LinterStatus
from snapcraft.meta import snap_yaml
from snapcraft.utils import process_version

if TYPE_CHECKING:
    from snapcraft.services import SnapcraftServiceFactory


class Package(PackageService):
    """Package service subclass for Snapcraft."""

    _project: models.Project

    def __init__(
        self,
        app: AppMetadata,
        services: SnapcraftServiceFactory,
        *,
        project: models.Project,
        platform: str | None,
        build_for: str,
    ) -> None:
        super().__init__(app, services, project=project)
        self._platform = platform
        self._build_for = build_for

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

        return [
            pathlib.Path(
                pack.pack_snap(
                    prime_dir,
                    output=str(dest),
                    compression=self._project.compression,
                    name=self._project.name,
                    version=process_version(self._project.version),
                    target_arch=self._project.get_build_for(),
                )
            )
        ]

    @override
    def write_metadata(self, path: pathlib.Path) -> None:
        """Write the project metadata to metadata.yaml in the given directory.

        :param path: The path to the prime directory.
        """
        meta_dir = self._services.lifecycle.prime_dir / "meta"
        meta_dir.mkdir(parents=True, exist_ok=True)

        self.metadata.to_yaml_file(meta_dir / "snap.yaml")

    @property
    def metadata(self) -> snap_yaml.SnapMetadata:
        """Get the metadata model for this project."""
        return snap_yaml.get_metadata_from_project(
            self._project, self._services.lifecycle.prime_dir, arch=self._build_for
        )
