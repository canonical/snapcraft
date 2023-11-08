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

import datetime
import pathlib
import typing
from typing import cast

from craft_application import AppMetadata, PackageService, models, util
from craft_cli import emit
from overrides import override

from snapcraft import linters, projects, errors, pack
from snapcraft.linters import LinterStatus
from snapcraft.meta import snap_yaml
from snapcraft.projects import SnapcraftProject
from snapcraft.utils import process_version

if typing.TYPE_CHECKING:
    from snapcraft.services import SnapcraftServiceFactory


class SnapcraftPackageService(PackageService):
    """Package service subclass for Snapcraft."""

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

        :param dest: Directory into which to write the package(s).
        :returns: A list of paths to created packages.
        """
        # TODO: Call snap pack
        project = cast(projects.SnapcraftProject, self._services.project)
        issues = linters.run_linters(prime_dir, lint=project.lint)
        status = linters.report(issues, intermediate=True)

        # In case of linter errors, stop execution and return the error code.
        if status in (LinterStatus.ERRORS, LinterStatus.FATAL):
            raise errors.LinterError("Linter errors found", exit_code=status)

        return [pathlib.Path(pack.pack_snap(
            prime_dir,
            output=dest,  # Blah this should be a path grrrr
            compression=project.compression,
            name=project.name,
            version=process_version(project.version),
            target_arch=project.get_build_for(),
        ))]

    @override
    def write_metadata(self, path: pathlib.Path) -> None:
        """Write the project metadata to metadata.yaml in the given directory.

        :param path: The path to the prime directory.
        """
        # TODO: Write any metadata files ("post-pack")
        # super() will write the model below into metadata.yaml in the prime dir
        project = cast(projects.SnapcraftProject, self._services.project)
        snap_yaml.write(project, path, arch=project.get_build_for())

    @property
    def metadata(self) -> models.BaseMetadata:
        """Get the metadata model for this project."""
        # TODO: create a base medatadaa model
