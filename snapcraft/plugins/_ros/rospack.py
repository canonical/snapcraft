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

import os
import logging
import subprocess
import tempfile
from typing import List, Set

import snapcraft
from snapcraft.internal import common, repo
from snapcraft import formatting_utils

logger = logging.getLogger(__name__)


class Rospack:
    def __init__(
        self,
        *,
        ros_distro: str,
        ros_package_path: str,
        rospack_path: str,
        ubuntu_sources: str,
        ubuntu_keyrings: List[str],
        project: snapcraft.project.Project
    ) -> None:
        """Create a new Rospack instance.

        :param str ros_distro: Name of the ROS distribution.
        :param str ros_package_path: The workspace to use.
        :param str rospack_path: Working directory for rospack (where it will be
                                 installed).
        :param str ubuntu_sources: Ubuntu repositories from which rospack will be
                                   installed.
        :param list ubuntu_keyrings: List of paths to keyrings to use for sources.
        :param project: Instance of Project for project-wide settings.
        :type project: snapcraft.Project
        """
        self._ros_distro = ros_distro
        self._ros_package_path = ros_package_path
        self._rospack_path = rospack_path
        self._ubuntu_sources = ubuntu_sources
        self._ubuntu_keyrings = ubuntu_keyrings
        self._project = project

        self._rospack_install_path = os.path.join(self._rospack_path, "install")
        self._rospack_cache_path = os.path.join(self._rospack_path, "cache")

    def setup(self) -> None:
        """Fetch, unpack, and setup rospack."""

        # Make sure we can run multiple times without error
        os.makedirs(self._rospack_install_path, exist_ok=True)

        # rospack isn't necessarily a dependency of the project, so we'll unpack
        # it off to the side and use it from there.
        logger.info("Preparing to fetch rospack...")
        ubuntu_repo = repo.Ubuntu(
            self._rospack_path,
            sources=self._ubuntu_sources,
            keyrings=self._ubuntu_keyrings,
            project_options=self._project,
        )

        logger.info("Fetching rospack...")
        ubuntu_repo.get(
            [
                "ros-{}-rospack".format(self._ros_distro),
                "ros-{}-catkin".format(self._ros_distro),
            ]
        )

        logger.info("Installing rospack...")
        ubuntu_repo.unpack(self._rospack_install_path)

    def list_names(self) -> Set[str]:
        """Obtain list of packages present in the workspace."""

        output = self._run(["list-names"]).strip()
        if output:
            return set(output.split("\n"))
        else:
            return set()

    def _run(self, arguments) -> str:
        with tempfile.NamedTemporaryFile(mode="w+") as f:
            lines = [
                'export PYTHONPATH="{}"'.format(
                    os.path.join(
                        self._rospack_install_path,
                        "usr",
                        "lib",
                        "python2.7",
                        "dist-packages",
                    )
                )
            ]

            ros_path = os.path.join(
                self._rospack_install_path, "opt", "ros", self._ros_distro
            )
            bin_paths = (
                os.path.join(ros_path, "bin"),
                os.path.join(self._rospack_install_path, "usr", "bin"),
            )
            lines.append(
                "export {}".format(
                    formatting_utils.format_path_variable(
                        "PATH", bin_paths, prepend="", separator=":"
                    )
                )
            )

            lib_paths = common.get_library_paths(
                self._rospack_install_path, self._project.arch_triplet
            )
            if lib_paths:
                lines.append(
                    "export {}".format(
                        formatting_utils.format_path_variable(
                            "LD_LIBRARY_PATH", lib_paths, prepend="", separator=":"
                        )
                    )
                )

            # Source our own workspace so we have all of rospack's dependencies
            lines.append(
                "_CATKIN_SETUP_DIR={} source {}".format(
                    ros_path, os.path.join(ros_path, "setup.sh")
                )
            )

            # By default, rospack saves its cache in $HOME/.ros, which we shouldn't
            # access here, so we'll redirect it with this environment variable.
            lines.append('export ROS_HOME="{}"'.format(self._rospack_cache_path))

            lines.append('export ROS_PACKAGE_PATH="{}"'.format(self._ros_package_path))

            lines.append('exec "$@"')
            f.write("\n".join(lines))
            f.flush()
            return (
                subprocess.check_output(
                    ["/bin/bash", f.name, "rospack"] + arguments,
                    stderr=subprocess.STDOUT,
                )
                .decode("utf8")
                .strip()
            )
