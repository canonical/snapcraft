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

import abc
import logging
import os
import shutil
from textwrap import dedent
from typing import List, Optional

from xdg import BaseDirectory

from . import errors
from ._snap import SnapInjector
from snapcraft.internal import steps


logger = logging.getLogger(__name__)


_CLOUD_USER_DATA_TMPL = dedent(
    """\
    #cloud-config
    manage_etc_hosts: true
    package_update: false
    growpart:
        mode: growpart
        devices: ["/"]
        ignore_growroot_disabled: false
    write_files:
        - path: /etc/skel/.bashrc
          permissions: 0644
          content: |
            export SNAPCRAFT_BUILD_ENVIRONMENT=managed-host
            export PS1="\h \$(/bin/_snapcraft_prompt)# "
            export PATH=/snap/bin:$PATH
        - path: /bin/_snapcraft_prompt
          permissions: 0755
          content: |
            #!/bin/bash
            if [[ "$PWD" =~ ^$HOME.* ]]; then
                path="${PWD/#$HOME/\ ..}"
                if [[ "$path" == " .." ]]; then
                    ps1=""
                else
                    ps1="$path"
                fi
            else
                ps1="$PWD"
            fi
            echo -n $ps1
    """
)


class Provider(abc.ABC):

    _SNAPS_MOUNTPOINT = os.path.join(os.path.sep, "var", "cache", "snapcraft", "snaps")
    _INSTANCE_PROJECT_DIR = "~/project"

    def __init__(self, *, project, echoer, is_ephemeral: bool = False) -> None:
        self.project = project
        self.echoer = echoer
        self._is_ephemeral = is_ephemeral

        self.instance_name = "snapcraft-{}".format(project.info.name)

        if project.info.version:
            self.snap_filename = "{}_{}_{}.snap".format(
                project.info.name, project.info.version, project.deb_arch
            )
        else:
            self.snap_filename = "{}_{}.snap".format(
                project.info.name, project.deb_arch
            )
        self.provider_project_dir = os.path.join(
            BaseDirectory.save_data_path("snapcraft"),
            "projects",
            self._get_provider_name(),
            project.info.name,
        )

    def __enter__(self):
        self.create()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.destroy()

    @abc.abstractclassmethod
    def _get_provider_name(cls) -> str:
        """Return the provider name."""

    @abc.abstractmethod
    def create(self) -> None:
        """Provider steps needed to create a fully functioning environment."""

    @abc.abstractmethod
    def destroy(self) -> None:
        """Provider steps needed to ensure the instance is destroyed.

        This method should be safe to call multiple times and do nothing
        if the instance to destroy is already destroyed.
        """

    @abc.abstractmethod
    def _run(self, command: List) -> None:
        """Run a command on the instance."""

    @abc.abstractmethod
    def _launch(self):
        """Launch the instance."""

    @abc.abstractmethod
    def _start(self):
        """Start an existing the instance."""

    @abc.abstractmethod
    def _push_file(self, *, source: str, destination: str) -> None:
        """Push a file into the instance."""

    @abc.abstractmethod
    def _mount(self, *, mountpoint: str, dev_or_path: str) -> None:
        """Mount a path from the host inside the instance."""

    @abc.abstractmethod
    def _umount(self, *, mountpoint: str) -> None:
        """Unmount the mountpoint from the instance."""

    @abc.abstractmethod
    def _mount_snaps_directory(self) -> None:
        """Mount the host directory with snaps into the provider."""

    @abc.abstractmethod
    def _unmount_snaps_directory(self) -> None:
        """Unmount the host directory with snaps from the provider."""

    @abc.abstractmethod
    def mount_project(self) -> None:
        """Provider steps needed to make the project available to the instance.
        """

    @abc.abstractmethod
    def provision_project(self, tarball: str) -> None:
        """Provider steps needed to copy project assests to the instance."""

    def execute_step(self, step: steps.Step) -> None:
        self._run(command=["snapcraft", step.name])

    def pack_project(self, *, output: Optional[str] = None) -> None:
        command = ["snapcraft", "snap"]
        if output:
            command.extend(["--output", output])
        self._run(command=command)

    def clean_project(self) -> bool:
        try:
            shutil.rmtree(self.provider_project_dir)
            return True
        except FileNotFoundError:
            return False

    @abc.abstractmethod
    def build_project(self) -> None:
        """Provider steps needed build the project on the instance."""

    @abc.abstractmethod
    def retrieve_snap(self) -> str:
        """
        Provider steps needed to retrieve the built snap from the instance.

        :returns: the filename of the retrieved snap.
        :rtype: str
        """

    @abc.abstractmethod
    def shell(self) -> None:
        """Provider steps to provide a shell into the instance."""

    def launch_instance(self) -> None:
        try:
            # An ProviderStartError exception here means we need to create
            self._start()
        except errors.ProviderInstanceNotFoundError:
            # If starting failed, we need to make sure our data directory is
            # clean
            if os.path.exists(self.provider_project_dir):
                shutil.rmtree(self.provider_project_dir)
            os.makedirs(self.provider_project_dir)
            # then launch
            self._launch()
            # We need to setup snapcraft now to be able to refresh
            self._setup_snapcraft()
            # and do first boot related things
            self._run(["snapcraft", "refresh"])
        else:
            # We always setup snapcraft after a start to bring it up to speed with
            # what is on the host
            self._setup_snapcraft()

    def _setup_snapcraft(self) -> None:
        registry_filepath = os.path.join(
            self.provider_project_dir, "snap-registry.yaml"
        )
        snap_injector = SnapInjector(
            snap_dir=self._SNAPS_MOUNTPOINT,
            registry_filepath=registry_filepath,
            snap_arch=self.project.deb_arch,
            runner=self._run,
            snap_dir_mounter=self._mount_snaps_directory,
            snap_dir_unmounter=self._unmount_snaps_directory,
            file_pusher=self._push_file,
        )
        # Inject snapcraft
        snap_injector.add(snap_name="core")
        snap_injector.add(snap_name="snapcraft")

        # Also inject the base when confinement is set to classic to have real
        # paths to the linker.
        if (
            self.project.info.base is not None
            and self.project.info.confinement == "classic"
        ):
            snap_injector.add(snap_name=self.project.info.base)

        snap_injector.apply()

    def _get_cloud_user_data(self) -> str:
        # TODO support users for the qemu provider.
        cloud_user_data_filepath = os.path.join(
            self.provider_project_dir, "user-data.yaml"
        )
        if os.path.exists(cloud_user_data_filepath):
            return cloud_user_data_filepath

        with open(cloud_user_data_filepath, "w") as cloud_user_data_file:
            print(_CLOUD_USER_DATA_TMPL, file=cloud_user_data_file)

        return cloud_user_data_filepath
