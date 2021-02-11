# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2021 Canonical Ltd
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
#
"""Manage the host's apt source repository configuration."""

import io
import logging
import os
import pathlib
import re
import subprocess
import tempfile
from typing import List, Optional

from snapcraft.internal import os_release
from snapcraft.internal.meta import package_repository
from snapcraft.project._project_options import ProjectOptions

from . import apt_ppa

logger = logging.getLogger(__name__)


def _construct_deb822_source(
    *,
    architectures: Optional[List[str]] = None,
    components: Optional[List[str]] = None,
    formats: Optional[List[str]] = None,
    suites: List[str],
    url: str,
) -> str:
    """Construct deb-822 formatted sources.list config string."""
    with io.StringIO() as deb822:
        if formats:
            type_text = " ".join(formats)
        else:
            type_text = "deb"

        print(f"Types: {type_text}", file=deb822)

        print(f"URIs: {url}", file=deb822)

        suites_text = " ".join(suites)
        print(f"Suites: {suites_text}", file=deb822)

        if components:
            components_text = " ".join(components)
            print(f"Components: {components_text}", file=deb822)

        if architectures:
            arch_text = " ".join(architectures)
        else:
            arch_text = _get_host_arch()

        print(f"Architectures: {arch_text}", file=deb822)

        return deb822.getvalue()


def _get_host_arch() -> str:
    return ProjectOptions().deb_arch


def _sudo_write_file(*, dst_path: pathlib.Path, content: bytes) -> None:
    """Workaround for writing privileged files in destructive mode."""
    try:
        with tempfile.NamedTemporaryFile(delete=False) as temp_file:
            temp_file.write(content)
            temp_file.flush()
            f_name = temp_file.name

        try:
            command = [
                "sudo",
                "install",
                "--owner=root",
                "--group=root",
                "--mode=0644",
                f_name,
                str(dst_path),
            ]
            subprocess.run(command, check=True)
        except subprocess.CalledProcessError as error:
            raise RuntimeError(
                f"Failed to install repository config with: {command!r}"
            ) from error
    finally:
        os.unlink(f_name)


class AptSourcesManager:
    """Manage apt source configuration in /etc/apt/sources.list.d.

    :param sources_list_d: Path to sources.list.d directory.
    """

    # pylint: disable=too-few-public-methods
    def __init__(
        self, *, sources_list_d: pathlib.Path = pathlib.Path("/etc/apt/sources.list.d"),
    ) -> None:
        self._sources_list_d = sources_list_d

    def _install_sources(
        self,
        *,
        architectures: Optional[List[str]] = None,
        components: Optional[List[str]] = None,
        formats: Optional[List[str]] = None,
        name: str,
        suites: List[str],
        url: str,
    ) -> bool:
        """Install sources list configuration.

        Write config to:
        /etc/apt/sources.list.d/snapcraft-<name>.sources

        :returns: True if configuration was changed.
        """
        config = _construct_deb822_source(
            architectures=architectures,
            components=components,
            formats=formats,
            suites=suites,
            url=url,
        )

        if name not in ["default", "default-security"]:
            name = "snapcraft-" + name

        config_path = self._sources_list_d / f"{name}.sources"
        if config_path.exists() and config_path.read_text() == config:
            # Already installed and matches, nothing to do.
            logger.debug("Ignoring unchanged sources: %s", str(config_path))
            return False

        _sudo_write_file(dst_path=config_path, content=config.encode())
        logger.debug("Installed sources: %s", str(config_path))
        return True

    def _install_sources_apt(
        self, *, package_repo: package_repository.PackageRepositoryApt
    ) -> bool:
        """Install repository configuration.

        1) First check to see if package repo is implied path,
           or "bare repository" config.  This is indicated when no
           path, components, or suites are indicated.
        2) If path is specified, convert path to a suite entry,
           ending with "/".

        Relatedly, this assumes all of the error-checking has been
        done already on the package_repository object in a proper
        fashion, but do some sanity checks here anyways.

        :returns: True if source configuration was changed.
        """
        if (
            not package_repo.path
            and not package_repo.components
            and not package_repo.suites
        ):
            suites = ["/"]
        elif package_repo.path:
            # Suites denoting exact path must end with '/'.
            path = package_repo.path
            if not path.endswith("/"):
                path += "/"
            suites = [path]
        elif package_repo.suites:
            suites = package_repo.suites
            if not package_repo.components:
                raise RuntimeError("no components with suite")
        else:
            raise RuntimeError("no suites or path")

        if package_repo.name:
            name = package_repo.name
        else:
            name = re.sub(r"\W+", "_", package_repo.url)

        return self._install_sources(
            architectures=package_repo.architectures,
            components=package_repo.components,
            formats=package_repo.formats,
            name=name,
            suites=suites,
            url=package_repo.url,
        )

    def _install_sources_ppa(
        self, *, package_repo: package_repository.PackageRepositoryAptPpa
    ) -> bool:
        """Install PPA formatted repository.

        Create a sources list config by:
        - Looking up the codename of the host OS and using it as the "suites"
          entry.
        - Formulate deb URL to point to PPA.
        - Enable only "deb" formats.

        :returns: True if source configuration was changed.
        """
        owner, name = apt_ppa.split_ppa_parts(ppa=package_repo.ppa)
        codename = os_release.OsRelease().version_codename()

        return self._install_sources(
            components=["main"],
            formats=["deb"],
            name=f"ppa-{owner}_{name}",
            suites=[codename],
            url=f"http://ppa.launchpad.net/{owner}/{name}/ubuntu",
        )

    def install_package_repository_sources(
        self, *, package_repo: package_repository.PackageRepository,
    ) -> bool:
        """Install configured package repositories.

        :param package_repo: Repository to install the source configuration for.

        :returns: True if source configuration was changed.
        """
        logger.debug("Processing repo: %r", package_repo)
        if isinstance(package_repo, package_repository.PackageRepositoryAptPpa):
            return self._install_sources_ppa(package_repo=package_repo)

        if isinstance(package_repo, package_repository.PackageRepositoryApt):
            return self._install_sources_apt(package_repo=package_repo)

        raise RuntimeError(f"unhandled package repository: {package_repository!r}")
