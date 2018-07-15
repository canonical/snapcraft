# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import contextlib
import os
import logging
import shutil
import subprocess

from snapcraft import repo, sources

logger = logging.getLogger(__name__)

_ROS2_URL_TEMPLATE = "https://raw.githubusercontent.com/ros2/ros2/{version}/ros2.repos"


_INSTALL_TOOLS_STEP = "install-tools"
_FETCH_ROS2_STEP = "fetch-ros2"
_BUILD_ROS2_STEP = "build-ros2"


class Bootstrapper:
    """Bootstrap ROS2 by building the underlay from source."""

    def __init__(self, *, version, bootstrap_path, ubuntu_sources, project):
        """Initialize bootstrapper.

        :param str version: The ROS2 version to boostrap (e.g. release-beta3)
        :param str bootstrap_path: Working directory for the bootstrap process
        :param str ubuntu_sources: Ubuntu repositories from which dependencies
                                   will be fetched.
        :param project: Instance of ProjectOptions for project-wide settings
        :type project: snapcraft.ProjectOptions
        """
        self._version = version
        self._ubuntu_sources = ubuntu_sources
        self._project = project

        self._bootstrap_path = bootstrap_path
        self._tool_dir = os.path.join(self._bootstrap_path, "tools")
        self._state_dir = os.path.join(self._bootstrap_path, "state")
        self._underlay_dir = os.path.join(self._bootstrap_path, "underlay")
        self._install_dir = os.path.join(self._bootstrap_path, "install")
        self._build_dir = os.path.join(self._bootstrap_path, "build")
        self._source_dir = os.path.join(self._underlay_dir, "src")

    def get_build_packages(self):
        """Return the packages required for building the underlay."""

        return [
            # Dependencies for FastRTPS
            "libasio-dev",
            "libtinyxml2-dev",
            # Dependencies for the rest of ros2
            "cmake",
            "libopencv-dev",
            "libpoco-dev",
            "libpocofoundation9v5",
            "libpocofoundation9v5-dbg",
            "python3-dev",
            "python3-empy",
            "python3-nose",
            "python3-pip",
            "python3-setuptools",
            "python3-yaml",
            "libtinyxml-dev",
            "libeigen3-dev",
        ]

    def get_stage_packages(self):
        """Return the packages required for running the underlay."""

        # Ament is a python3 tool, and it requires setuptools at runtime
        return ["python3", "python3-setuptools"]

    def pull(self):
        """Download the ROS2 underlay.

        This first downloads the tools required to fetch the underlay, and then
        uses those tools to fetch the underlay. Both steps are only run once,
        even if subsequent steps fail.
        """
        self._run_step(
            self._install_tools,
            step=_INSTALL_TOOLS_STEP,
            skip_message="Tools already installed. Skipping...",
        )
        self._run_step(
            self._fetch_ros2,
            step=_FETCH_ROS2_STEP,
            skip_message="ros2 already fetched. Skipping...",
        )

    def build(self):
        """Build the ROS2 underlay.

        Build the ROS2 underlay that was fetched in pull(). This is only done
        once, assuming success, which means calling this function multiple
        times does not necessarily build multiple times.

        :return: The path into which the underlay is installed.
        :rtype: str
        """
        self._run_step(
            self._build_ros2,
            step=_BUILD_ROS2_STEP,
            skip_message="ros2 already built. Skipping...",
        )

        return self._install_dir

    def clean(self):
        """Clean everything done in this class."""

        with contextlib.suppress(FileNotFoundError):
            shutil.rmtree(self._bootstrap_path)

    def _run(self, command):
        env = os.environ.copy()
        env["PATH"] = env["PATH"] + ":" + os.path.join(self._tool_dir, "usr", "bin")
        env["PYTHONPATH"] = os.path.join(
            self._tool_dir, "usr", "lib", "python3", "dist-packages"
        )

        subprocess.check_call(command, env=env)

    def _is_step_done(self, step):
        return os.path.isfile(os.path.join(self._state_dir, step))

    def _set_step_done(self, step):
        os.makedirs(self._state_dir, exist_ok=True)
        open(os.path.join(self._state_dir, step), "w").close()

    def _run_step(self, callable, *, step, skip_message=None):
        if self._is_step_done(step):
            if skip_message:
                logger.debug(skip_message)
        else:
            callable()
            self._set_step_done(step)

    def _install_tools(self):
        logger.info("Preparing to fetch vcstool...")
        ubuntu = repo.Ubuntu(
            self._bootstrap_path,
            sources=self._ubuntu_sources,
            project_options=self._project,
        )

        logger.info("Fetching vcstool...")
        ubuntu.get(["python3-vcstool"])

        logger.info("Installing vcstool...")
        ubuntu.unpack(self._tool_dir)

    def _fetch_ros2(self):
        os.makedirs(self._source_dir, exist_ok=True)
        sources.Script(
            _ROS2_URL_TEMPLATE.format(version=self._version), self._underlay_dir
        ).download()

        logger.info("Fetching ros2 sources....")
        ros2_repos = os.path.join(self._underlay_dir, "ros2.repos")
        self._run(["vcs", "import", "--input", ros2_repos, self._source_dir])

    def _build_ros2(self):
        logger.info("Building ros2 underlay...")
        ament_path = os.path.join(
            self._source_dir, "ament", "ament_tools", "scripts", "ament.py"
        )
        self._run(
            [
                ament_path,
                "build",
                self._source_dir,
                "--build-space",
                self._build_dir,
                "--install-space",
                self._install_dir,
            ]
        )
