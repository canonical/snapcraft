# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

import logging
from subprocess import PIPE, STDOUT, CalledProcessError, run
from typing import List

from snapcraft.internal.meta.package_management import PackageManagement, Repository
from snapcraft.internal.os_release import OsRelease
from . import errors
from . import Repo

logger = logging.getLogger(__name__)


def _verify_supported_distribution() -> None:
    # While it is likely that other debian-based distros will work as-is, this
    # has only been tested against Ubuntu bases.  This code should never be
    # reached if package-management is not configured, ensuring safety for
    # those building with non-ubuntu bases.
    os_name = OsRelease().id()
    if os_name != "ubuntu":
        raise RuntimeError(
            f"unable to install repository for unsupported OS: {os_name}"
        )


def _apt_key_decoder_ring(output: bytes, repo: Repository) -> str:
    message: str = output.decode()
    message = message.replace(
        "Warning: apt-key output should not be parsed (stdout is not a terminal)", ""
    ).strip()

    # Improve error messages that we can.
    if "gpg: keyserver receive failed: No data" in message:
        message = f"GPG key ID {repo.gpg_public_key_id!r} not found on key server {repo.gpg_key_server!r}"
    elif "gpg: keyserver receive failed: Server indicated a failure" in message:
        message = (
            f"unable to establish connection to key server {repo.gpg_key_server!r}"
        )
    elif "gpg: keyserver receive failed: Connection timed out" in message:
        message = f"unable to establish connection to key server {repo.gpg_key_server!r} (connection timed out)"

    return message


def _install_apt_repository_key_id(repo: Repository) -> None:
    if not repo.gpg_public_key_id:
        return

    # Set default key-server if none specified.
    if not repo.gpg_key_server:
        repo.gpg_key_server = "keyserver.ubuntu.com"

    cmd = [
        "sudo",
        "apt-key",
        "adv",
        "--keyserver",
        repo.gpg_key_server,
        "--recv-keys",
        repo.gpg_public_key_id,
    ]

    try:
        run(cmd, stdout=PIPE, stderr=STDOUT, check=True)
    except CalledProcessError as error:
        message = _apt_key_decoder_ring(error.output, repo)
        raise errors.RepositoryKeyError(repo=repo, message=message) from error

    logger.debug(
        f"Installed apt repository GPG key ID {repo.gpg_public_key_id!r} from key server {repo.gpg_key_server!r}"
    )


def _install_apt_repository_key_block(repo: Repository) -> None:
    if not repo.gpg_public_key:
        return

    cmd = ["sudo", "apt-key", "add", "-"]
    try:
        run(
            cmd,
            input=repo.gpg_public_key.encode(),
            stdout=PIPE,
            stderr=STDOUT,
            check=True,
        )
    except CalledProcessError as error:
        message = _apt_key_decoder_ring(error.output, repo)
        raise errors.RepositoryKeyError(repo=repo, message=message) from error

    logger.debug(f"Installed apt repository key:\n{repo.gpg_public_key}")


def _install_apt_repository(repo: Repository) -> None:
    cmd = ["sudo", "apt-add-repository", "-y", repo.source]
    try:
        run(cmd, stdout=PIPE, stderr=STDOUT, check=True)
    except CalledProcessError as error:
        raise errors.RepositoryKeyError(repo=repo, message=error.output) from error

    logger.debug(f"Installed apt repository {repo.source!r}")


def _get_repo_build_tools() -> List[str]:
    return ["software-properties-common"]


def _install_repository(repo: Repository) -> None:
    if repo.gpg_public_key:
        _install_apt_repository_key_block(repo)

    if repo.gpg_public_key_id:
        _install_apt_repository_key_id(repo)

    _install_apt_repository(repo)


def configure_package_manager(package_management: PackageManagement) -> List[str]:
    """Configures package manager according to snapcraft.yaml configuration.

    :returns: a list of installed build packages
    """
    if not package_management.repositories:
        return list()

    # There are repositories to configure, assert that we are
    # using a supported platform.
    _verify_supported_distribution()

    # Install required build tools.
    repo_build_tools = _get_repo_build_tools()
    if repo_build_tools:
        installed_tools = Repo.install_build_packages(repo_build_tools)

    # Install repositories.
    for repository in package_management.repositories:
        _install_repository(repository)

    return installed_tools
