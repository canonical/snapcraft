# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2019-2022 Canonical Ltd.
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

"""Package repository installer."""

import pathlib
from typing import Any, Dict, List

from . import errors
from .apt_key_manager import AptKeyManager
from .apt_sources_manager import AptSourcesManager
from .package_repository import (
    PackageRepository,
    PackageRepositoryApt,
    PackageRepositoryAptPPA,
)


def install(
    project_repositories: List[Dict[str, Any]], *, key_assets: pathlib.Path
) -> bool:
    """Add package repositories to the host system.

    :param package_repositories: A list of package repositories to install.
    :param key_assets: The directory containing repository keys.

    :return: Whether a package list refresh is required.
    """
    key_manager = AptKeyManager(key_assets=key_assets)
    sources_manager = AptSourcesManager()

    package_repositories = _unmarshal_repositories(project_repositories)

    refresh_required = False
    for package_repo in package_repositories:
        refresh_required |= key_manager.install_package_repository_key(
            package_repo=package_repo
        )
        refresh_required |= sources_manager.install_package_repository_sources(
            package_repo=package_repo
        )

    _verify_all_key_assets_installed(key_assets=key_assets, key_manager=key_manager)

    return refresh_required


def _verify_all_key_assets_installed(
    *,
    key_assets: pathlib.Path,
    key_manager: AptKeyManager,
) -> None:
    """Verify all configured key assets are utilized, error if not."""
    for key_asset in key_assets.glob("*"):
        key = key_asset.read_text()
        for key_id in key_manager.get_key_fingerprints(key=key):
            if not key_manager.is_key_installed(key_id=key_id):
                raise errors.PackageRepositoryError(
                    "Found unused key asset {key_asset!r}.",
                    details="All configured key assets must be utilized.",
                    resolution="Verify key usage and remove all unused keys.",
                )


def _unmarshal_repositories(
    project_repositories: List[Dict[str, Any]]
) -> List[PackageRepository]:
    """Create package repositories objects from project data."""
    repositories = []
    for data in project_repositories:
        pkg_repo: PackageRepository

        if "ppa" in data:
            pkg_repo = PackageRepositoryAptPPA.unmarshal(data)
        else:
            pkg_repo = PackageRepositoryApt.unmarshal(data)

        repositories.append(pkg_repo)

    return repositories
