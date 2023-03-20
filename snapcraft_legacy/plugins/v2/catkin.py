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

from snapcraft_legacy.internal.repo.snaps import _get_parsed_snap
from snapcraft_legacy.plugins.v2 import _ros


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

    def _get_source_command(self, path: str) -> List[str]:
        return [
            f'if [ -f "{path}/opt/ros/${{ROS_DISTRO}}/setup.sh" ]; then',
            'set -- --local "${_EXTEND_WS}"',
            '_CATKIN_SETUP_DIR="{fpath}" . "{fpath}/setup.sh"'.format(
                fpath=f"{path}/opt/ros/${{ROS_DISTRO}}"
            ),
            'if [ -z ${_EXTEND_WS} ]; then _EXTEND_WS="--extend"; fi',
            "fi",
        ]

    def _get_workspace_activation_commands(self) -> List[str]:
        """Return a list of commands to source a ROS workspace.

        The commands returned will be run before doing anything else.
        They will be run in a single shell instance with the rest of
        the build step, so these commands can affect the commands that
        follow.

        snapcraftctl can be used in the script to call out to snapcraft
        specific functionality.
        """

        activation_commands = list()

        # Source ROS ws in all build-snaps first
        activation_commands.append("## Sourcing ROS ws in build snaps")
        if self.options.build_snaps:
            for build_snap in self.options.build_snaps:
                snap_name = _get_parsed_snap(build_snap)[0]
                activation_commands.extend(self._get_source_command('/snap/{build_snap}/current'.format(build_snap=snap_name)))
            activation_commands.append("")

        # Source ROS ws in stage-snaps next
        activation_commands.append("## Sourcing ROS ws in stage snaps")
        activation_commands.extend(self._get_source_command("${SNAPCRAFT_PART_INSTALL}"))
        activation_commands.append("")

        # Finally source system's ROS ws
        activation_commands.append("## Sourcing ROS ws in system")
        activation_commands.extend(self._get_source_command(""))
        activation_commands.append("")

        return activation_commands

    def _get_build_commands(self) -> List[str]:
        cmd = [
            "catkin_make_isolated",
            "--install",
            "--merge",
            "--source-space",
            '"${SNAPCRAFT_PART_SRC_WORK}"',
            "--build-space",
            '"${SNAPCRAFT_PART_BUILD}"',
            "--install-space",
            '"${SNAPCRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}"',
            "-j",
            '"${SNAPCRAFT_PARALLEL_BUILD_COUNT}"',
        ]

        if self.options.catkin_packages:
            cmd.extend(["--pkg", *self.options.catkin_packages])

        if self.options.catkin_packages_ignore:
            cmd.extend(["--ignore-pkg", *self.options.catkin_packages_ignore])

        if self.options.catkin_cmake_args:
            cmd.extend(["--cmake-args", *self.options.catkin_cmake_args])

        return ["## Build command", " ".join(cmd)]
