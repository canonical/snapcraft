# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2015-2022 Canonical Ltd.
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

"""APT key management helpers."""

import pathlib
import subprocess
import tempfile
from typing import List, Optional

import gnupg
from craft_cli import emit

from . import apt_ppa, errors, package_repository


class AptKeyManager:
    """Manage APT repository keys."""

    def __init__(
        self,
        *,
        gpg_keyring: pathlib.Path = pathlib.Path(
            "/etc/apt/trusted.gpg.d/snapcraft.gpg"
        ),
        key_assets: pathlib.Path,
    ) -> None:
        self._gpg_keyring = gpg_keyring
        self._key_assets = key_assets

    def find_asset_with_key_id(self, *, key_id: str) -> Optional[pathlib.Path]:
        """Find snap key asset matching key_id.

        The key asset much be named with the last 8 characters of the key
        identifier, in upper case.

        :param key_id: Key ID to search for.

        :returns: Path of key asset if match found, otherwise None.
        """
        key_file = key_id[-8:].upper() + ".asc"
        key_path = self._key_assets / key_file

        if key_path.exists():
            return key_path

        return None

    @classmethod
    def get_key_fingerprints(cls, *, key: str) -> List[str]:
        """List fingerprints found in specified key.

        Do this by importing the key into a temporary keyring,
        then querying the keyring for fingerprints.

        :param key: Key data (string) to parse.

        :returns: List of key fingerprints/IDs.
        """
        with tempfile.NamedTemporaryFile(suffix="keyring") as temp_file:
            return (
                gnupg.GPG(keyring=temp_file.name).import_keys(key_data=key).fingerprints
            )

    @classmethod
    def is_key_installed(cls, *, key_id: str) -> bool:
        """Check if specified key_id is installed.

        Check if key is installed by attempting to export the key.
        Unfortunately, apt-key does not exit with error and
        we have to do our best to parse the output.

        :param key_id: Key ID to check for.

        :returns: True if key is installed.
        """
        try:
            proc = subprocess.run(
                ["apt-key", "export", key_id],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=True,
            )
        except subprocess.CalledProcessError as error:
            # Export shouldn't exit with failure based on testing,
            # but assume the key is not installed and log a warning.
            emit.progress(f"Unexpected apt-key failure: {error.output}", permanent=True)
            return False

        apt_key_output = proc.stdout.decode()

        if "BEGIN PGP PUBLIC KEY BLOCK" in apt_key_output:
            return True

        if "nothing exported" in apt_key_output:
            return False

        # The two strings above have worked in testing, but if neither is
        # present for whatever reason, assume the key is not installed
        # and log a warning.
        emit.progress(f"Unexpected apt-key output: {apt_key_output}", permanent=True)
        return False

    def install_key(self, *, key: str) -> None:
        """Install given key.

        :param key: Key to install.

        :raises: AptGPGKeyInstallError if unable to install key.
        """
        cmd = [
            "apt-key",
            "--keyring",
            str(self._gpg_keyring),
            "add",
            "-",
        ]

        try:
            emit.debug(f"Executing: {cmd!r}")
            env = {}
            env["LANG"] = "C.UTF-8"
            subprocess.run(
                cmd,
                input=key.encode(),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=True,
                env=env,
            )
        except subprocess.CalledProcessError as error:
            raise errors.AptGPGKeyInstallError(error.output.decode(), key=key)

        emit.debug(f"Installed apt repository key:\n{key}")

    def install_key_from_keyserver(
        self, *, key_id: str, key_server: str = "keyserver.ubuntu.com"
    ) -> None:
        """Install key from specified key server.

        :param key_id: Key ID to install.
        :param key_server: Key server to query.

        :raises: AptGPGKeyInstallError if unable to install key.
        """
        env = {}
        env["LANG"] = "C.UTF-8"

        cmd = [
            "apt-key",
            "--keyring",
            str(self._gpg_keyring),
            "adv",
            "--keyserver",
            key_server,
            "--recv-keys",
            key_id,
        ]

        try:
            emit.debug(f"Executing: {cmd!r}")
            subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                check=True,
                env=env,
            )
        except subprocess.CalledProcessError as error:
            raise errors.AptGPGKeyInstallError(
                error.output.decode(), key_id=key_id, key_server=key_server
            )

    def install_package_repository_key(
        self, *, package_repo: package_repository.PackageRepository
    ) -> bool:
        """Install required key for specified package repository.

        For both PPA and other Apt package repositories:
        1) If key is already installed, return False.
        2) Install key from local asset, if available.
        3) Install key from key server, if available. An unspecified
           keyserver will default to using keyserver.ubuntu.com.

        :param package_repo: Apt PackageRepository configuration.

        :returns: True if key configuration was changed. False if
            key already installed.

        :raises: AptGPGKeyInstallError if unable to install key.
        """
        key_server: Optional[str] = None
        if isinstance(package_repo, package_repository.PackageRepositoryAptPPA):
            key_id = apt_ppa.get_launchpad_ppa_key_id(ppa=package_repo.ppa)
        elif isinstance(package_repo, package_repository.PackageRepositoryApt):
            key_id = package_repo.key_id
            key_server = package_repo.key_server
        else:
            raise RuntimeError(f"unhandled package repo type: {package_repo!r}")

        # Already installed, nothing to do.
        if self.is_key_installed(key_id=key_id):
            return False

        key_path = self.find_asset_with_key_id(key_id=key_id)
        if key_path is not None:
            self.install_key(key=key_path.read_text())
        else:
            if key_server is None:
                key_server = "keyserver.ubuntu.com"
            self.install_key_from_keyserver(key_id=key_id, key_server=key_server)

        return True
