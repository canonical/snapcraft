# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

"""The colcon plugin for ROS2 parts.

    - colcon-packages:
      (list of strings)
      List of colcon packages to build. If not specified, all packages in the
      workspace will be built. If set to an empty list ([]), no packages will
      be built, which could be useful if you only want ROS debs in the snap.

    - colcon-packages-ignore:
      (list of strings)
      List of colcon packages to ignore. If not specified or set to an empty
      list ([]), no packages will be ignored.

    - colcon-cmake-args:
      (list of strings)
      Arguments to pass to cmake projects. Note that any arguments here which match
      colcon arguments need to be prefixed with a space. This can be done by quoting
      each argument with a leading space.

    - colcon-catkin-cmake-args:
      (list of strings)
      Arguments to pass to catkin packages. Note that any arguments here which match
      colcon arguments need to be prefixed with a space. This can be done by quoting
      each argument with a leading space.

    - colcon-ament-cmake-args:
      (list of strings)
      Arguments to pass to ament_cmake packages. Note that any arguments here which
      match colcon arguments need to be prefixed with a space. This can be done by
      quoting each argument with a leading space.

    This plugin expects the build-environment `ROS_DISTRO` and `ROS_BUILD_BASE`
    to be populated by the `ros2-<distro>` extension.

    This plugin also expects certain variables that are specified by the extension,
    specific to the ROS distro.  If not using the extension, set these in your
    `build-environment`:
      - ROS_DISTRO: "foxy"
"""

from typing import Any, Dict, List, Set

from snapcraft.plugins.v2 import _ros


class ColconPlugin(_ros.RosPlugin):
    @classmethod
    def get_schema(cls) -> Dict[str, Any]:
        return {
            "$schema": "http://json-schema.org/draft-04/schema#",
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "colcon-ament-cmake-args": {
                    "type": "array",
                    "minItems": 0,
                    "items": {"type": "string"},
                    "default": [],
                },
                "colcon-catkin-cmake-args": {
                    "type": "array",
                    "minItems": 0,
                    "items": {"type": "string"},
                    "default": [],
                },
                "colcon-cmake-args": {
                    "type": "array",
                    "minItems": 0,
                    "items": {"type": "string"},
                    "default": [],
                },
                "colcon-packages": {
                    "type": "array",
                    "minItems": 0,
                    "uniqueItems": True,
                    "items": {"type": "string"},
                },
                "colcon-packages-ignore": {
                    "type": "array",
                    "minItems": 0,
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                },
            },
        }

    def get_build_packages(self) -> Set[str]:
        return super().get_build_packages() | {
            "python3-colcon-common-extensions",
            "python3-rosinstall",
            "python3-wstool",
        }

    def get_build_environment(self) -> Dict[str, str]:
        env = super().get_build_environment()
        env.update(
            {
                "AMENT_PYTHON_EXECUTABLE": "/usr/bin/python3",
                "COLCON_PYTHON_EXECUTABLE": "/usr/bin/python3",
            }
        )

        return env

    def _get_workspace_activation_commands(self) -> List[str]:
        """Return a list of commands source a ROS 2 workspace.

        The commands returned will be run before doing anything else.
        They will be run in a single shell instance with the rest of
        the build step, so these commands can affect the commands that
        follow.

        snapcraftctl can be used in the script to call out to snapcraft
        specific functionality.
        """

        # There are a number of unbound vars, disable flag
        # after saving current state to restore after.
        return [
            'state="$(set +o)"',
            "set +u",
            # If it exists, source the stage-snap underlay
            'if [ -f "${SNAPCRAFT_PART_INSTALL}"/opt/ros/snap/setup.sh ]; then',
            "COLCON_CURRENT_PREFIX={path} . {path}/setup.sh".format(
                path='"${SNAPCRAFT_PART_INSTALL}"/opt/ros/snap'
            ),
            "fi",
            '. /opt/ros/"${ROS_DISTRO}"/local_setup.sh',
            'eval "${state}"',
        ]

    def _get_build_commands(self) -> List[str]:
        cmd = [
            "colcon",
            "build",
            "--base-paths",
            '"${SNAPCRAFT_PART_SRC_WORK}"',
            "--build-base",
            '"${SNAPCRAFT_PART_BUILD}"',
            "--merge-install",
            "--install-base",
            '"${SNAPCRAFT_PART_INSTALL}"/opt/ros/snap',
        ]

        if self.options.colcon_packages_ignore:
            cmd.extend(["--packages-ignore", *self.options.colcon_packages_ignore])

        if self.options.colcon_packages:
            cmd.extend(["--packages-select", *self.options.colcon_packages])

        if self.options.colcon_cmake_args:
            cmd.extend(["--cmake-args", *self.options.colcon_cmake_args])

        if self.options.colcon_ament_cmake_args:
            cmd.extend(["--ament-cmake-args", *self.options.colcon_ament_cmake_args])

        if self.options.colcon_catkin_cmake_args:
            cmd.extend(["--catkin-cmake-args", *self.options.colcon_catkin_cmake_args])

        # Specify the number of workers
        cmd.extend(["--parallel-workers", '"${SNAPCRAFT_PARALLEL_BUILD_COUNT}"'])

        return [" ".join(cmd)] + [
            # Remove the COLCON_IGNORE marker so that, at staging,
            # catkin can crawl the entire folder to look up for packages.
            'if [ -f "${SNAPCRAFT_PART_INSTALL}"/opt/ros/snap/COLCON_IGNORE ]; then',
            'rm "${SNAPCRAFT_PART_INSTALL}"/opt/ros/snap/COLCON_IGNORE',
            "fi",
        ]
