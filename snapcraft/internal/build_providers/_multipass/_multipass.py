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

import os
import shlex

from .. import errors
from .._base_provider import Provider
from ._instance_info import InstanceInfo
from ._multipass_command import MultipassCommand


class Multipass(Provider):
    """A multipass provider for snapcraft to execute its lifecycle."""

    @classmethod
    def _get_provider_name(cls):
        return "multipass"

    def _run(self, command, hide_output: bool = False) -> None:
        self._multipass_cmd.execute(
            instance_name=self.instance_name, command=command, hide_output=hide_output
        )

    def _launch(self) -> None:
        try:
            # An exception here means we need to create
            self._multipass_cmd.start(instance_name=self.instance_name)
        except errors.ProviderStartError:
            cloud_user_data_filepath = self._get_cloud_user_data()

            self._multipass_cmd.launch(
                instance_name=self.instance_name,
                image="16.04",
                cloud_init=cloud_user_data_filepath,
            )

    def _mount(self, *, mountpoint: str, dev_or_path: str) -> None:
        target = "{}:{}".format(self.instance_name, mountpoint)
        self._multipass_cmd.mount(source=dev_or_path, target=target)

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
        if self._instance_info is None:
            return

        if not self._instance_info.is_stopped():
            self._multipass_cmd.stop(instance_name=self.instance_name)
        if self._is_ephemeral:
            self.clean_project()

    def mount_project(self) -> None:
        # Resolve the home directory
        home_dir = (
            self._multipass_cmd.execute(
                command=["pwd"], hide_output=True, instance_name=self.instance_name
            )
            .decode()
            .strip()
        )
        project_mountpoint = os.path.join(home_dir, "project")

        # multipass keeps the mount active, so check if it is there first.
        if not self._instance_info.is_mounted(project_mountpoint):
            self._mount(
                mountpoint=project_mountpoint, dev_or_path=self.project.project_dir
            )

    def provision_project(self, tarball: str) -> None:
        """Provision the multipass instance with the project to work with."""
        # TODO add instance check.
        # Step 0, sanitize the input
        tarball = shlex.quote(tarball)

        # First create a working directory
        self._multipass_cmd.execute(
            command=["mkdir", self._INSTANCE_PROJECT_DIR],
            instance_name=self.instance_name,
        )

        # Then copy the tarball over
        destination = "{}:{}".format(self.instance_name, tarball)
        self._multipass_cmd.copy_files(source=tarball, destination=destination)

        # Finally extract it into project_dir.
        extract_cmd = ["tar", "-xvf", tarball, "-C", self._INSTANCE_PROJECT_DIR]
        self._multipass_cmd.execute(
            command=extract_cmd, instance_name=self.instance_name
        )

    def clean_project(self) -> bool:
        was_cleaned = super().clean_project()
        if was_cleaned:
            self._multipass_cmd.delete(instance_name=self.instance_name)
        return was_cleaned

    def build_project(self) -> None:
        # TODO add instance check.
        self._multipass_cmd.execute(
            command=["snapcraft", "snap", "--output", self.snap_filename],
            instance_name=self.instance_name,
        )

    def retrieve_snap(self) -> str:
        # TODO add instance check.
        source = "{}:{}/{}".format(
            self.instance_name, self._INSTANCE_PROJECT_DIR, self.snap_filename
        )
        self._multipass_cmd.copy_files(source=source, destination=self.snap_filename)
        return self.snap_filename

    def _get_instance_info(self):
        instance_info_raw = self._multipass_cmd.info(
            instance_name=self.instance_name, output_format="json"
        )
        return InstanceInfo.from_json(
            instance_name=self.instance_name, json_info=instance_info_raw.decode()
        )
