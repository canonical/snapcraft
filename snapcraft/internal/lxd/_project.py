# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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
import logging
import shutil
import subprocess
from typing import List

from . import errors
from ._containerbuild import Containerbuild
from snapcraft.project import Project as _Project
from snapcraft.internal import lifecycle, steps

logger = logging.getLogger(__name__)


class Project(Containerbuild):
    def __init__(self, *, output: str, source: str, project: _Project) -> None:
        super().__init__(
            output=output,
            source=source,
            project=project,
            container_name=project.info.name,
            remote="local",
        )

    def _ensure_container(self):
        if not self._get_container_status():
            try:
                subprocess.check_call(
                    ["lxc", "init", self._image, self._container_name]
                )
            except subprocess.CalledProcessError as e:
                raise errors.ContainerCreationFailedError() from e
        if self._get_container_status()["status"] == "Stopped":
            self._configure_container()
            try:
                subprocess.check_call(["lxc", "start", self._container_name])
            except subprocess.CalledProcessError:
                raise errors.ContainerStartFailedError()
        self._wait_for_network()
        self._wait_for_cloud_init()
        self._inject_snapcraft()

    def _configure_container(self):
        super()._configure_container()
        # Remove existing device (to ensure we update old containers)
        devices = self._get_container_status()["devices"]
        if self._project_folder in devices:
            subprocess.check_call(
                [
                    "lxc",
                    "config",
                    "device",
                    "remove",
                    self._container_name,
                    self._project_folder,
                ]
            )
        if "fuse" not in devices:
            subprocess.check_call(
                [
                    "lxc",
                    "config",
                    "device",
                    "add",
                    self._container_name,
                    "fuse",
                    "unix-char",
                    "path=/dev/fuse",
                ]
            )

    def _setup_project(self):
        devices = self._get_container_status()["devices"]
        if self._project_folder not in devices:
            logger.info("Mounting {} into container".format(self._source))
            subprocess.check_call(
                [
                    "lxc",
                    "config",
                    "device",
                    "add",
                    self._container_name,
                    self._project_folder,
                    "disk",
                    "source={}".format(self._source),
                    "path={}".format(self._project_folder),
                ]
            )

    def refresh(self):
        with self._container_running():
            self._container_run(["apt-get", "update"])
            self._container_run(["apt-get", "upgrade", "-y"])
            self._container_run(["snap", "refresh"])

    def clean(self, parts: List[str], step: steps.Step):
        # clean with no parts deletes the container
        if not step:
            if not parts:
                if os.path.exists(self.provider_project_dir):
                    shutil.rmtree(self.provider_project_dir)
                self._ensure_remote()
                if self._get_container_status():
                    print("Deleting {}".format(self._container_name))
                    subprocess.check_call(["lxc", "delete", "-f", self._container_name])
            step = steps.PULL

        lifecycle.clean(self._project, parts, step)
