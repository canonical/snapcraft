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

"""The catkin plugin for ROS 1 parts.

    - catkin-packages:
      (list of strings)
      List of catkin packages to build. If not specified, all packages in the
      workspace will be built. If set to an empty list ([]), no packages will
      be built, which could be useful if you only want ROS debs in the snap.

    - catkin-packages-ignore:
      (list of strings)
      List of catkin packages to ignore (i.e. not build or install). If not
      specified or set to an empty list ([]), no packages will be ignored.

    - catkin-cmake-args:
      (list of strings)
      Arguments to pass to cmake projects.

    This plugin requires certain variables that are specified by the `ros1-noetic`
    extension. If you're not using the extension, set these in your `build-environment`:
      - ROS_DISTRO: "noetic"
"""

from typing import Any, Dict, List, Set

from snapcraft.plugins.v2 import _ros


class CatkinPlugin(_ros.RosPlugin):
    @classmethod
    def get_schema(cls) -> Dict[str, Any]:
        return {
            "$schema": "http://json-schema.org/draft-04/schema#",
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "catkin-cmake-args": {
                    "type": "array",
                    "minItems": 0,
                    "items": {"type": "string"},
                    "default": [],
                },
                "catkin-packages": {
                    "type": "array",
                    "minItems": 0,
                    "uniqueItems": True,
                    "items": {"type": "string"},
                },
                "catkin-packages-ignore": {
                    "type": "array",
                    "minItems": 0,
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                },
            },
        }

    def get_build_packages(self) -> Set[str]:
        return super().get_build_packages() | {"ros-noetic-catkin"}

    def _get_build_commands(self) -> List[str]:
        cmd = [
            "catkin_make_isolated",
            "--install",
            "--merge",
            "--source-space",
            "$SNAPCRAFT_PART_SRC",
            "--build-space",
            "$SNAPCRAFT_PART_BUILD",
            "--install-space",
            "$SNAPCRAFT_PART_INSTALL/opt/ros/$ROS_DISTRO",
            "-j",
            "$SNAPCRAFT_PARALLEL_BUILD_COUNT",
        ]

        if self.options.catkin_packages:
            cmd.extend(["--pkg", *self.options.catkin_packages])

        if self.options.catkin_packages_ignore:
            cmd.extend(["--ignore-pkg", *self.options.catkin_packages_ignore])

        if self.options.catkin_cmake_args:
            cmd.extend(["--cmake-args", *self.options.catkin_cmake_args])

        return [" ".join(cmd)]

    def _get_workspace_activation_commands(self) -> List[str]:
        return [
            'state="$(set +o)"',
            "set +u",
            "_CATKIN_SETUP_DIR=/opt/ros/$ROS_DISTRO . /opt/ros/$ROS_DISTRO/setup.sh",
            'eval "$(state)"',
        ]
