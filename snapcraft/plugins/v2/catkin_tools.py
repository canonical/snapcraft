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

"""The catkin tools plugin for ROS 1 parts.

    - catkin-tools-packages:
      (list of strings)
      List of catkin packages to build. If not specified, all packages in the
      workspace will be built. If set to an empty list ([]), no packages will
      be built, which could be useful if you only want ROS debs in the snap.

    - catkin-tools-cmake-args:
      (list of strings)
      Arguments to pass to cmake projects.

    This plugin requires certain variables that are specified by the `ros1-noetic`
    extension. If you're not using the extension, set these in your `build-environment`:
      - ROS_DISTRO: "noetic"
"""

from typing import Any, Dict, List, Set

from snapcraft.plugins.v2 import _ros


class CatkinToolsPlugin(_ros.RosPlugin):
    @classmethod
    def get_schema(cls) -> Dict[str, Any]:
        return {
            "$schema": "http://json-schema.org/draft-04/schema#",
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "catkin-tools-cmake-args": {
                    "type": "array",
                    "minItems": 0,
                    "items": {"type": "string"},
                    "default": [],
                },
                "catkin-tools-packages": {
                    "type": "array",
                    "minItems": 0,
                    "uniqueItems": True,
                    "items": {"type": "string"},
                },
            },
        }

    def get_build_packages(self) -> Set[str]:
        return super().get_build_packages() | {
            "python3-catkin-tools",
            # FIXME: Only needed because of a botched release:
            # https://github.com/catkin/catkin_tools/issues/594#issuecomment-688149976
            # Once fixed, remove this.
            "python3-osrf-pycommon",
        }

    def _get_build_commands(self) -> List[str]:
        # It's possible that this workspace wasn't initialized to be used with
        # catkin-tools, so initialize it first. Note that this is a noop if it
        # was already initialized
        commands = ["catkin init"]

        # Overwrite the default catkin profile to ensure builds
        # aren't affected by profile changes
        commands.append("catkin profile add -f default")

        # Use catkin config to set configurations for the snap build
        catkin_config_command = [
            "catkin",
            "config",
            "--profile",
            "default",
            "--install",
            "--source-space",
            "$SNAPCRAFT_PART_SRC",
            "--build-space",
            "$SNAPCRAFT_PART_BUILD",
            "--install-space",
            "$SNAPCRAFT_PART_INSTALL/opt/ros/$ROS_DISTRO",
        ]

        if self.options.catkin_tools_cmake_args:
            catkin_config_command.extend(
                ["--cmake-args", *self.options.catkin_tools_cmake_args]
            )

        commands.append(" ".join(catkin_config_command))

        # Now actually build the package
        catkin_command = [
            "catkin",
            "build",
            "--no-notify",
            "--profile",
            "default",
            "-j",
            "$SNAPCRAFT_PARALLEL_BUILD_COUNT",
        ]

        if self.options.catkin_tools_packages:
            catkin_command.extend(self.options.catkin_tools_packages)

        commands.append(" ".join(catkin_command))

        return commands

    def _get_workspace_activation_commands(self) -> List[str]:
        return [
            'state="$(set +o)"',
            "set +u",
            "_CATKIN_SETUP_DIR=/opt/ros/$ROS_DISTRO . /opt/ros/$ROS_DISTRO/setup.sh",
            'eval "$(state)"',
        ]
