#!/usr/bin/python3
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
import shutil
import subprocess

import petname

from . import errors
from ._containerbuild import Containerbuild
from snapcraft.project import Project


class Cleanbuilder(Containerbuild):
    def __init__(self, *, output=None, source, project: Project, remote=None) -> None:
        container_name = petname.Generate(3, "-")
        super().__init__(
            output=output,
            source=source,
            project=project,
            container_name=container_name,
            remote=remote,
        )

    def execute(self):
        super().execute()

    def _ensure_container(self):
        try:
            subprocess.check_call(
                ["lxc", "launch", "-e", self._image, self._container_name]
            )
        except subprocess.CalledProcessError as e:
            raise errors.ContainerCreationFailedError() from e
        self._configure_container()
        self._wait_for_network()
        self._wait_for_cloud_init()
        self._container_run(["apt-get", "update"])
        self._inject_snapcraft()

    def _setup_project(self):
        tar_filename = self._source
        # os.sep needs to be `/` and on Windows it will be set to `\`
        dst = "{}/{}".format(self._project_folder, os.path.basename(tar_filename))
        self._container_run(["mkdir", self._project_folder])
        self._lxd_instance.push_file(tar_filename, dst)
        self._container_run(
            ["tar", "xvf", os.path.basename(tar_filename)], cwd=self._project_folder
        )

    def _finish(self, success=True):
        super()._finish(success=success)

        # remove project data as this is a cleanbuild
        if os.path.exists(self.provider_project_dir):
            shutil.rmtree(self.provider_project_dir)

        if success:
            # os.sep needs to be `/` and on Windows it will be set to `\`
            src = "{}/{}".format(self._project_folder, self.snap_filename)
            self._lxd_instance.pull_file(src, self.snap_filename)
