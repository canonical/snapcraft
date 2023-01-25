# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

This plugin expects the build-environment `ROS_VERSION` and `ROS_DISTRO`
to be populated by the `ros2-<distro>` extension.

This plugin also expects certain variables that are specified by the extension,
specific to the ROS distro. If not using the extension, set these in your
    `build-environment`:
      - ROS_VERSION: "2"
      - ROS_DISTRO: "humble"
"""

from typing import Any, Dict, List, Set, cast

from craft_parts import plugins
from overrides import overrides

from . import _ros


class ColconPluginProperties(plugins.PluginProperties, plugins.PluginModel):
    """The part properties used by the Colcon plugin."""

    colcon_ament_cmake_args: List[str] = []
    colcon_catkin_cmake_args: List[str] = []
    colcon_cmake_args: List[str] = []
    colcon_packages: List[str] = []
    colcon_packages_ignore: List[str] = []

    # part properties required by the plugin
    source: str

    @classmethod
    def unmarshal(cls, data: Dict[str, Any]) -> "ColconPluginProperties":
        """Populate make properties from the part specification.

        :param data: A dictionary containing part properties.

        :return: The populated plugin properties data object.

        :raise pydantic.ValidationError: If validation fails.
        """
        plugin_data = plugins.extract_plugin_properties(
            data, plugin_name="colcon", required=["source"]
        )
        return cls(**plugin_data)


class ColconPlugin(_ros.RosPlugin):
    """Plugin for the colcon build tool."""

    properties_class = ColconPluginProperties

    @overrides
    def get_build_packages(self) -> Set[str]:
        return super().get_build_packages() | {
            "python3-colcon-common-extensions",
            "python3-rosinstall",
            "python3-wstool",
        }

    @overrides
    def get_build_environment(self) -> Dict[str, str]:
        env = super().get_build_environment()
        env.update(
            {
                "AMENT_PYTHON_EXECUTABLE": "/usr/bin/python3",
                "COLCON_PYTHON_EXECUTABLE": "/usr/bin/python3",
            }
        )

        return env

    @overrides
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
            'state="$(set +o); set -$-"',
            "set +u",
            # If it exists, source the stage-snap underlay
            'if [ -f "${CRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}/local_setup.sh" ]; then',
            'COLCON_CURRENT_PREFIX="{path}" . "{path}/local_setup.sh"'.format(
                path="${CRAFT_PART_INSTALL}/opt/ros/${ROS_DISTRO}"
            ),
            "fi",
            'if [ -f "${CRAFT_PART_INSTALL}/opt/ros/snap/local_setup.sh" ]; then',
            'COLCON_CURRENT_PREFIX="{path}" . "{path}/local_setup.sh"'.format(
                path="${CRAFT_PART_INSTALL}/opt/ros/snap"
            ),
            "fi",
            '. "/opt/ros/${ROS_DISTRO}/local_setup.sh"',
            'eval "${state}"',
        ]

    @overrides
    def _get_build_commands(self) -> List[str]:
        options = cast(ColconPluginProperties, self._options)

        cmd = [
            "colcon",
            "build",
            "--base-paths",
            '"${CRAFT_PART_SRC_WORK}"',
            "--build-base",
            '"${CRAFT_PART_BUILD}"',
            "--merge-install",
            "--install-base",
            '"${CRAFT_PART_INSTALL}/opt/ros/snap"',
        ]

        if options.colcon_packages_ignore:
            cmd.extend(["--packages-ignore", *options.colcon_packages_ignore])

        if options.colcon_packages:
            cmd.extend(["--packages-select", *options.colcon_packages])

        if options.colcon_cmake_args:
            cmd.extend(["--cmake-args", *options.colcon_cmake_args])

        if options.colcon_ament_cmake_args:
            cmd.extend(["--ament-cmake-args", *options.colcon_ament_cmake_args])

        if options.colcon_catkin_cmake_args:
            cmd.extend(["--catkin-cmake-args", *options.colcon_catkin_cmake_args])

        # Specify the number of workers
        cmd.extend(["--parallel-workers", '"${CRAFT_PARALLEL_BUILD_COUNT}"'])

        return [" ".join(cmd)] + [
            # Remove the COLCON_IGNORE marker so that, at staging,
            # catkin can crawl the entire folder to look up for packages.
            'if [ -f "${CRAFT_PART_INSTALL}/opt/ros/snap/COLCON_IGNORE" ]; then',
            'rm "${CRAFT_PART_INSTALL}/opt/ros/snap/COLCON_IGNORE"',
            "fi",
        ]
