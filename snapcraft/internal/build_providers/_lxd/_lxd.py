# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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
import os
import subprocess
import sys
import urllib.parse
import warnings
from typing import Optional, Sequence

import pylxd

from .._base_provider import Provider
from .._base_provider import errors
from ._images import get_image_source
from snapcraft.internal import repo
from snapcraft.internal.errors import SnapcraftEnvironmentError


logger = logging.getLogger(__name__)
# Filter out attribute setting warnings for properties that exist in LXD operations
# but are unhandled in pylxd.
warnings.filterwarnings("ignore", module="pylxd.models.operation")


class LXD(Provider):
    """A LXD provider for snapcraft to execute its lifecycle."""

    _PROJECT_DEVICE_NAME = "snapcraft-project"
    _PROJECT_EXPORTED_PRIME_NAME = "snapcraft-project-prime"

    # Given that we are running snapcraft from the snapcraft snap, which has
    # classic confinement and require using the lxd snap, the lxd and lxc
    # binaries should be found in /snap/bin
    _LXD_BIN = os.path.join(os.path.sep, "snap", "bin", "lxd")
    _LXC_BIN = os.path.join(os.path.sep, "snap", "bin", "lxc")

    @classmethod
    def ensure_provider(cls):
        error_message = None  # type: Optional[str]
        prompt_installable = False

        if sys.platform != "linux":
            error_message = "LXD is not supported on this platform"
        else:
            try:
                if not repo.snaps.SnapPackage.is_snap_installed("lxd"):
                    error_message = (
                        "The LXD snap is required to continue: snap install lxd"
                    )
                    prompt_installable = True
            except repo.errors.SnapdConnectionError:
                error_message = (
                    "snap support is required to continue: "
                    "https://docs.snapcraft.io/installing-snapd/6735"
                )

        if error_message is not None:
            raise errors.ProviderNotFound(
                provider=cls._get_provider_name(),
                prompt_installable=prompt_installable,
                error_message=error_message,
            )

        # If we reach this point, it means the lxd snap is properly setup.
        # Now is the time for additional sanity checks to ensure the provider
        # will work.
        try:
            # TODO: add support for more distributions. Maybe refactor a bit so that Repo behaves
            # similar to a build provider.
            if repo.Repo.is_package_installed("lxd") or repo.Repo.is_package_installed(
                "lxd-client"
            ):
                raise SnapcraftEnvironmentError(
                    (
                        "The {!r} provider does not support having the 'lxd' or "
                        "'lxd-client' deb packages installed. To completely migrate "
                        "to the LXD snap run 'lxd.migrate' and try again."
                    ).format(cls._get_provider_name())
                )
        except repo.errors.NoNativeBackendError:
            pass

    @classmethod
    def setup_provider(cls, *, echoer) -> None:
        repo.snaps.install_snaps(["lxd/latest/stable"])

        try:
            subprocess.check_output([cls._LXD_BIN, "waitready", "--timeout=30"])
        except subprocess.CalledProcessError as call_error:
            raise SnapcraftEnvironmentError(
                "Timeout reached waiting for LXD to start."
            ) from call_error

        try:
            subprocess.check_output([cls._LXD_BIN, "init", "--auto"])
        except subprocess.CalledProcessError as call_error:
            raise SnapcraftEnvironmentError(
                "Failed to initialize LXD. "
                "Try manually initializing before trying again: lxd init --auto."
            ) from call_error

    @classmethod
    def _get_provider_name(cls):
        return "LXD"

    @classmethod
    def _get_is_snap_injection_capable(cls) -> bool:
        return True

    def _run(
        self, command: Sequence[str], hide_output: bool = False
    ) -> Optional[bytes]:
        self._ensure_container_running()

        logger.debug("Running {}".format(" ".join(command)))
        # TODO: use pylxd
        cmd = [self._LXC_BIN, "exec", self.instance_name, "--"] + list(command)
        try:
            if hide_output:
                output = subprocess.check_output(cmd)
            else:
                # output here will be None, and that is OK.
                output = subprocess.check_call(cmd)
        except subprocess.CalledProcessError as process_error:
            raise errors.ProviderExecError(
                provider_name=self._get_provider_name(),
                command=command,
                exit_code=process_error.returncode,
            ) from process_error

        return output

    def _launch(self) -> None:
        config = {
            "name": self.instance_name,
            "source": get_image_source(base=self.project.info.get_build_base()),
        }

        try:
            container = self._lxd_client.containers.create(config, wait=True)
        except pylxd.exceptions.LXDAPIException as lxd_api_error:
            raise errors.ProviderLaunchError(
                provider_name=self._get_provider_name(), error_message=lxd_api_error
            ) from lxd_api_error
        container.config["user.user-data"] = self._get_cloud_user_data_string()
        # This is setup by cloud init, but set it here to be on the safer side.
        container.config["environment.SNAPCRAFT_BUILD_ENVIRONMENT"] = "managed-host"
        container.save(wait=True)
        self._container = container

        self._start()

    def _start(self):
        if not self._lxd_client.containers.exists(self.instance_name):
            raise errors.ProviderInstanceNotFoundError(instance_name=self.instance_name)

        if self._container is None:
            self._container = self._lxd_client.containers.get(self.instance_name)

        self._container.sync()
        self._container.config["environment.SNAPCRAFT_HAS_TTY"] = str(
            sys.stdout.isatty()
        )
        # map to the owner of the directory we are eventually going to write the
        # snap to.
        self._container.config["raw.idmap"] = "both {!s} 0".format(
            os.stat(self.project._project_dir).st_uid
        )
        self._container.save(wait=True)

        if self._container.status.lower() != "running":
            try:
                self._container.start(wait=True)
            except pylxd.exceptions.LXDAPIException as lxd_api_error:
                print(self._container.status)
                raise errors.ProviderStartError(
                    provider_name=self._get_provider_name(), error_message=lxd_api_error
                ) from lxd_api_error

        # Ensure cloud init is done
        self.echoer.wrapped("Waiting for cloud-init")
        self._run(command=["cloud-init", "status", "--wait"])

    def _stop(self):
        # If _container is still None here it means creation/starting was not
        # successful.
        if self._container is None:
            return

        self._container.sync()

        if self._container.status.lower() != "stopped":
            try:
                self._container.stop(wait=True)
            except pylxd.exceptions.LXDAPIException as lxd_api_error:
                raise errors.ProviderStopError(
                    provider_name=self._get_provider_name(), error_message=lxd_api_error
                ) from lxd_api_error

    def _push_file(self, *, source: str, destination: str) -> None:
        self._ensure_container_running()

        # TODO: better handling of larger files.
        with open(source, "rb") as source_data:
            source_contents = source_data.read()

        try:
            self._container.files.put(destination, source_contents)
        except pylxd.exceptions.LXDAPIException as lxd_api_error:
            raise errors.ProviderFileCopyError(
                provider_name=self._get_provider_name(), error_message=lxd_api_error
            )

    def __init__(self, *, project, echoer, is_ephemeral: bool = False) -> None:
        super().__init__(project=project, echoer=echoer, is_ephemeral=is_ephemeral)
        self.echoer.warning(
            "The LXD provider is offered as a technology preview for early adopters.\n"
            "The command line interface, container names or lifecycle handling may "
            "change in upcoming releases."
        )
        # This endpoint is hardcoded everywhere lxc/lxd-pkg-snap#33
        lxd_socket_path = "/var/snap/lxd/common/lxd/unix.socket"
        endpoint = "http+unix://{}".format(urllib.parse.quote(lxd_socket_path, safe=""))
        try:
            self._lxd_client = pylxd.Client(endpoint=endpoint)
        except pylxd.client.exceptions.ClientConnectionFailed:
            raise errors.ProviderCommunicationError(
                provider_name=self._get_provider_name(),
                message="cannot connect to the LXD socket ({!r}).".format(
                    lxd_socket_path
                ),
            )
        self._container = None  # type: Optional[pylxd.models.container.Container]

    def create(self) -> None:
        """Create the LXD instance and setup the build environment."""
        self.echoer.info("Launching a container.")
        self.launch_instance()

    def destroy(self) -> None:
        """Destroy the instance, trying to stop it first."""
        self._stop()

    def mount_project(self) -> None:
        if self._PROJECT_DEVICE_NAME in self._container.devices:
            return

        # Resolve the home directory
        home_dir = (
            self._run(command=["printenv", "HOME"], hide_output=True).decode().strip()
        )

        self._container.sync()

        self._container.devices[self._PROJECT_DEVICE_NAME] = {
            "type": "disk",
            "source": self.project._project_dir,
            "path": os.path.join(home_dir, "project"),
        }

        try:
            self._container.save(wait=True)
        except pylxd.exceptions.LXDAPIException as lxd_api_error:
            raise errors.ProviderMountError(
                provider_name=self._get_provider_name(), error_message=lxd_api_error
            ) from lxd_api_error

    def _mount_prime_directory(self) -> bool:
        if self._PROJECT_EXPORTED_PRIME_NAME in self._container.devices:
            return True

        # Resolve the home directory
        home_dir = (
            self._run(command=["printenv", "HOME"], hide_output=True).decode().strip()
        )

        self._container.sync()

        self._container.devices[self._PROJECT_EXPORTED_PRIME_NAME] = {
            "type": "disk",
            "source": os.path.join(self.project.prime_dir),
            "path": os.path.join(home_dir, "prime"),
        }

        try:
            self._container.save(wait=True)
        except pylxd.exceptions.LXDAPIException as lxd_api_error:
            raise errors.ProviderMountError(
                provider_name=self._get_provider_name(), error_message=lxd_api_error
            ) from lxd_api_error

        return False

    def clean_project(self) -> bool:
        was_cleaned = super().clean_project()
        if was_cleaned:
            if self._container is None:
                self._container = self._lxd_client.containers.get(self.instance_name)
            self._stop()
            self._container.delete(wait=True)
        return was_cleaned

    def pull_file(self, name: str, destination: str, delete: bool = False) -> None:
        self._ensure_container_running()

        # TODO: better handling of larger files.
        try:
            source_data = self._container.files.get(name)
        except pylxd.exceptions.LXDAPIException as lxd_api_error:
            raise errors.ProviderFileCopyError(
                provider_name=self._get_provider_name(), error_message=lxd_api_error
            )
        else:
            with open(destination, "wb") as destination_data:
                destination_data.write(source_data)
        if delete and self._container.files.delete_available:
            self._container.files.delete(name)
        if delete and not self._container.files.delete_available:
            logger.warning("File deletion not supported by this LXD version.")

    def shell(self) -> None:
        self._run(command=["/bin/bash"])

    def _ensure_container_running(self) -> None:
        self._container.sync()
        if self._container.status.lower() != "running":
            raise errors.ProviderFileCopyError(
                provider_name=self._get_provider_name(),
                error_message=(
                    "Container is not running, the current state is: {!r]. "
                    "Ensure it has not been modified by external factors and try again"
                ).format(self._container.status),
            )
