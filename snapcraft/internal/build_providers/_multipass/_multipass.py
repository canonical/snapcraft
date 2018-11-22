# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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
import shlex
from typing import Dict, Sequence

from .. import errors
from .._base_provider import Provider
from ._instance_info import InstanceInfo
from ._multipass_command import MultipassCommand
from snapcraft.internal.errors import SnapcraftEnvironmentError


logger = logging.getLogger(__name__)


def _get_stop_time() -> int:
    try:
        timeout = int(os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT_STOP_TIME", "10"))
    except ValueError:
        raise SnapcraftEnvironmentError(
            "'SNAPCRAFT_BUILD_ENVIRONMENT_STOP_TIME' is incorrectly set, found {!r} "
            "but expected a number representing the minutes to delay the actual stop "
            "operation (or 0 to stop immediately).".format(
                os.getenv("SNAPCRAFT_BUILD_ENVIRONMENT_STOP_TIME")
            )
        )

    return timeout


class _MachineSetting:
    def __init__(self, *, envvar: str, default: str) -> None:
        self._default = default
        self._envvar = envvar

    def get_value(self):
        value = os.getenv(self._envvar, self._default)
        if value != self._default:
            logger.warning(
                "{!r} was set to {!r} in the environment. "
                "Changing the default allocation upon user request".format(
                    self._envvar, value
                )
            )
        return value


class Multipass(Provider):
    """A multipass provider for snapcraft to execute its lifecycle."""

    @classmethod
    def _get_provider_name(cls):
        return "multipass"

    def _run(self, command: Sequence[str], hide_output: bool = False) -> None:
        command = ["sudo", "-i"] + list(command)
        self._multipass_cmd.execute(
            instance_name=self.instance_name, command=command, hide_output=hide_output
        )

    def _get_disk_image(self) -> str:
        if self.project.info.base is None:
            image = "snapcraft:core16"
        else:
            image = "snapcraft:{}".format(self.project.info.base)

        return image

    def _launch(self) -> None:
        cloud_user_data_filepath = self._get_cloud_user_data()
        image = self._get_disk_image()

        cpus = _MachineSetting(envvar="SNAPCRAFT_BUILD_ENVIRONMENT_CPU", default="2")
        mem = _MachineSetting(envvar="SNAPCRAFT_BUILD_ENVIRONMENT_MEMORY", default="2G")
        disk = _MachineSetting(
            envvar="SNAPCRAFT_BUILD_ENVIRONMENT_DISK", default="256G"
        )

        self._multipass_cmd.launch(
            instance_name=self.instance_name,
            cpus=cpus.get_value(),
            mem=mem.get_value(),
            disk=disk.get_value(),
            image=image,
            cloud_init=cloud_user_data_filepath,
        )

    def _start(self):
        try:
            self._get_instance_info()
        except errors.ProviderInfoError as instance_error:
            # Until we have proper multipass error codes to know if this
            # was a communication error we should keep this error tracking
            # and generation here.
            raise errors.ProviderInstanceNotFoundError(
                instance_name=self.instance_name
            ) from instance_error

        self._multipass_cmd.start(instance_name=self.instance_name)

    def _mount(
        self,
        *,
        mountpoint: str,
        dev_or_path: str,
        uid_map: Dict[str, str] = None,
        gid_map: Dict[str, str] = None
    ) -> None:
        target = "{}:{}".format(self.instance_name, mountpoint)
        self._multipass_cmd.mount(
            source=dev_or_path, target=target, uid_map=uid_map, gid_map=gid_map
        )

    def _umount(self, *, mountpoint: str) -> None:
        mount = "{}:{}".format(self.instance_name, mountpoint)
        self._multipass_cmd.umount(mount=mount)

    def _mount_snaps_directory(self) -> None:
        # https://github.com/snapcore/snapd/blob/master/dirs/dirs.go
        # CoreLibExecDir
        path = os.path.join(os.path.sep, "var", "lib", "snapd", "snaps")
        self._mount(mountpoint=self._SNAPS_MOUNTPOINT, dev_or_path=path)

    def _unmount_snaps_directory(self):
        self._umount(mountpoint=self._SNAPS_MOUNTPOINT)

    def _push_file(self, *, source: str, destination: str) -> None:
        destination = "{}:{}".format(self.instance_name, destination)
        self._multipass_cmd.copy_files(source=source, destination=destination)

    def __init__(self, *, project, echoer, is_ephemeral: bool = False) -> None:
        super().__init__(project=project, echoer=echoer, is_ephemeral=is_ephemeral)
        self._multipass_cmd = MultipassCommand()
        self._instance_info = None  # type: InstanceInfo

    def create(self) -> None:
        """Create the multipass instance and setup the build environment."""
        self.launch_instance()
        self._instance_info = self._get_instance_info()

    def destroy(self) -> None:
        """Destroy the instance, trying to stop it first."""
        try:
            instance_info = self._instance_info = self._get_instance_info()
        except errors.ProviderInfoError:
            return

        if instance_info.is_stopped():
            return

        stop_time = _get_stop_time()
        if stop_time > 0:
            try:
                self._multipass_cmd.stop(
                    instance_name=self.instance_name, time=stop_time
                )
            except errors.ProviderStopError:
                self._multipass_cmd.stop(instance_name=self.instance_name)
        else:
            self._multipass_cmd.stop(instance_name=self.instance_name)

        if self._is_ephemeral:
            self.clean_project()

    def mount_project(self) -> None:
        # Resolve the home directory
        home_dir = (
            self._multipass_cmd.execute(
                command=["sudo", "-i", "printenv", "HOME"],
                hide_output=True,
                instance_name=self.instance_name,
            )
            .decode()
            .strip()
        )
        project_mountpoint = os.path.join(home_dir, "project")

        # multipass keeps the mount active, so check if it is there first.
        if not self._instance_info.is_mounted(project_mountpoint):
            self._mount(
                mountpoint=project_mountpoint,
                dev_or_path=self.project._project_dir,
                uid_map={str(os.getuid()): "0"},
                gid_map={str(os.getgid()): "0"},
            )

    def provision_project(self, tarball: str) -> None:
        """Provision the multipass instance with the project to work with."""
        # TODO add instance check.
        # Step 0, sanitize the input
        tarball = shlex.quote(tarball)

        # First create a working directory
        self._multipass_cmd.execute(
            command=["sudo", "-i", "mkdir", self._INSTANCE_PROJECT_DIR],
            instance_name=self.instance_name,
        )

        # Then copy the tarball over
        destination = "{}:{}".format(self.instance_name, tarball)
        self._multipass_cmd.copy_files(source=tarball, destination=destination)

        # Finally extract it into project_dir.
        extract_cmd = [
            "sudo",
            "-i",
            "tar",
            "-xvf",
            tarball,
            "-C",
            self._INSTANCE_PROJECT_DIR,
        ]
        self._multipass_cmd.execute(
            command=extract_cmd, instance_name=self.instance_name
        )

    def clean_project(self) -> bool:
        was_cleaned = super().clean_project()
        if was_cleaned:
            self._multipass_cmd.delete(instance_name=self.instance_name, purge=True)
        return was_cleaned

    def build_project(self) -> None:
        # TODO add instance check.
        self._multipass_cmd.execute(
            command=["sudo", "-i", "snapcraft", "snap", "--output", self.snap_filename],
            instance_name=self.instance_name,
        )

    def retrieve_snap(self) -> str:
        # TODO add instance check.
        source = "{}:{}/{}".format(
            self.instance_name, self._INSTANCE_PROJECT_DIR, self.snap_filename
        )
        self._multipass_cmd.copy_files(source=source, destination=self.snap_filename)
        return self.snap_filename

    def shell(self) -> None:
        self._multipass_cmd.execute(
            instance_name=self.instance_name, command=["sudo", "-i", "/bin/bash"]
        )

    def _get_instance_info(self):
        instance_info_raw = self._multipass_cmd.info(
            instance_name=self.instance_name, output_format="json"
        )
        return InstanceInfo.from_json(
            instance_name=self.instance_name, json_info=instance_info_raw.decode()
        )
