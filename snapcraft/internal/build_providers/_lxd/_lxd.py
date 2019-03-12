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
from typing import Optional, Sequence

import pylxd

from .._base_provider import Provider
from .._base_provider import errors
from ._images import get_image_source


logger = logging.getLogger(__name__)


class LXD(Provider):
    """A LXD provider for snapcraft to execute its lifecycle."""

    _PROJECT_DEVICE_NAME = "snapcraft-project"

    @classmethod
    def _get_provider_name(cls):
        return "LXD"

    @classmethod
    def _get_is_snap_injection_capable(cls) -> bool:
        return False

    def _run(
        self, command: Sequence[str], hide_output: bool = False
    ) -> Optional[bytes]:
        self._ensure_container_running()

        logger.debug("Running {}".format(" ".join(command)))
        cmd = ["lxc", "exec", self.instance_name, "--"] + list(command)
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
            "source": get_image_source(base=self.project.info.base),
        }

        try:
            container = self._lxd_client.containers.create(config, wait=True)
        except pylxd.exceptions.LXDAPIException as lxd_api_error:
            raise errors.ProviderLaunchError(
                provider_name=self._get_provider_name(), error_message=lxd_api_error
            ) from lxd_api_error
        container.config["raw.idmap"] = "both {!s} 0".format(os.getuid())
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
        self._container.sync()

        if self._container.status.lower() != "stopped":
            try:
                self._container.stop(wait=True)
            except pylxd.exceptions.LXDAPIException as lxd_api_error:
                print(self._container.status)
                raise errors.ProviderStopError(
                    provider_name=self._get_provider_name(), error_message=lxd_api_error
                ) from lxd_api_error

    def _mount_snaps_directory(self) -> None:
        raise NotImplementedError(
            "Feature not supported with provider {!r}".format(self._get_provider_name())
        )

    def _unmount_snaps_directory(self):
        raise NotImplementedError(
            "Feature not supported with provider {!r}".format(self._get_provider_name())
        )

    def _push_file(self, *, source: str, destination: str) -> None:
        self._ensure_container_running()

        # TODO: better handling of larger files.
        with open(source, "wb") as source_data:
            try:
                self._container.files.put(destination, source_data.read())
            except pylxd.exceptions.LXDAPIException as lxd_api_error:
                raise errors.ProviderFileCopyError(
                    provider_name=self._get_provider_name(), error_message=lxd_api_error
                )

    def __init__(self, *, project, echoer, is_ephemeral: bool = False) -> None:
        super().__init__(project=project, echoer=echoer, is_ephemeral=is_ephemeral)
        self.echoer.warning(
            "The LXD provider is offered as a technology preview for early adopters. "
            "The command line interface, container names or lifecycle handling may "
            "change in upcoming releases."
        )
        self._lxd_client = pylxd.Client()
        self._container = None  # type: Optional[pylxd.models.container.Container]

    def create(self) -> None:
        """Create the LXD instance and setup the build environment."""
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
                error_message="container is not running, it is current state is: {!r].".format(
                    self._container.status
                ),
            )
