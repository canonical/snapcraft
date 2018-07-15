# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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
import subprocess
import sys
from typing import List

import snapcraft
from snapcraft.internal import errors, repo

logger = logging.getLogger(__name__)


class WstoolError(errors.SnapcraftError):
    pass


class WorkspaceInitializationError(WstoolError):
    fmt = "Error initializing workspace: {message}"

    def __init__(self, message: str) -> None:
        super().__init__(message=message)


class RosinstallMergeError(WstoolError):
    fmt = "Error merging rosinstall file {path!r} into workspace: {message}"

    def __init__(self, path: str, message: str) -> None:
        super().__init__(path=path, message=message)


class WorkspaceUpdateError(WstoolError):
    fmt = "Error updating workspace: {message}"

    def __init__(self, message: str) -> None:
        super().__init__(message=message)


class Wstool:
    """This class serves as a Python wrapper for the CLI utility wstool."""

    def __init__(
        self,
        ros_package_path: str,
        wstool_path: str,
        ubuntu_sources: str,
        project: snapcraft.ProjectOptions,
    ) -> None:
        """Create new Wstool

        :param str ros_package_path: The path where the packages should be
                                     fetched.
        :param str wstool_path: Working directory for wstool (where it will be
                                installed).
        :param str ubuntu_sources: Ubuntu repositories from which wstool will
                                   be installed.
        :param project: Instance of ProjectOptions for project-wide settings.
        :type project: snapcraft.ProjectOptions
        """
        self._ros_package_path = ros_package_path
        self._ubuntu_sources = ubuntu_sources
        self._wstool_path = wstool_path
        self._wstool_install_path = os.path.join(wstool_path, "install")
        self._project = project

    def setup(self) -> None:
        """Download/install wstool, and initialize workspace."""

        os.makedirs(self._wstool_install_path, exist_ok=True)

        # wstool isn't a dependency of the project, so we'll unpack it
        # somewhere else, and use it from there.
        logger.info("Preparing to fetch wstool...")
        ubuntu = repo.Ubuntu(
            self._wstool_path,
            sources=self._ubuntu_sources,
            project_options=self._project,
        )
        logger.info("Fetching wstool...")
        ubuntu.get(["python-wstool"])

        logger.info("Installing wstool...")
        ubuntu.unpack(self._wstool_install_path)

        logger.info("Initializing workspace (if necessary)...")
        try:
            self._run(
                [
                    "init",
                    self._ros_package_path,
                    "-j{}".format(self._project.parallel_build_count),
                ]
            )
        except subprocess.CalledProcessError as e:
            output = e.output.decode(sys.getfilesystemencoding()).strip()
            if "already is a workspace" not in output:
                stderr = e.stderr.decode(sys.getfilesystemencoding()).strip()
                raise WorkspaceInitializationError(stderr)

    def merge(self, rosinstall_file: str) -> str:
        """Merge rosinstall file with existing rosinstall in workspace.

        :param str rosinstall_file: Path to rosinstall file to merge in.
        """

        try:
            # Summary of arguments used:
            # --confirm-all: Don't ask for confirmation
            # -t: Path to workspace
            return self._run(
                [
                    "merge",
                    rosinstall_file,
                    "--confirm-all",
                    "-t{}".format(self._ros_package_path),
                ]
            ).strip()
        except subprocess.CalledProcessError as e:
            stderr = e.stderr.decode(sys.getfilesystemencoding()).strip()
            raise RosinstallMergeError(rosinstall_file, stderr)

    def update(self) -> str:
        """Update workspace from rosinstall file.

        This actually hits the network and downloads all repositories in
        the workspace's rosinstall file.
        """
        try:
            # Summary of arguments used:
            # -j: Use multiple jobs to fetch
            # -t: Path to workspace (into which packages will be fetched)
            return self._run(
                [
                    "update",
                    "-j{}".format(self._project.parallel_build_count),
                    "-t{}".format(self._ros_package_path),
                ]
            )
        except subprocess.CalledProcessError as e:
            stderr = e.stderr.decode(sys.getfilesystemencoding()).strip()
            raise WorkspaceUpdateError(stderr)

    def _run(self, arguments: List[str]) -> str:
        env = os.environ.copy()

        env["PATH"] += ":" + os.path.join(self._wstool_install_path, "usr", "bin")
        env["PYTHONPATH"] = os.path.join(
            self._wstool_install_path, "usr", "lib", "python2.7", "dist-packages"
        )

        return (
            subprocess.check_output(
                ["wstool"] + arguments, stderr=subprocess.PIPE, env=env
            )
            .decode(sys.getfilesystemencoding())
            .strip()
        )
