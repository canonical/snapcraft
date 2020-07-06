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

import click
import os
import pathlib
import shutil
import subprocess
import sys
from typing import Any, Dict, List, Set

from catkin_pkg import packages as catkin_packages

from snapcraft.internal.repo import Ubuntu
from snapcraft.plugins.v2 import PluginV2
from snapcraft.plugins.v1._ros.rosdep import _parse_rosdep_resolve_dependencies


def _get_python_command(env: Dict[str, str], command: List[str]) -> List[str]:
    env["PYTHONPATH"] = ":".join([p for p in sys.path if p])

    env_flags = [f"{key}={value}" for key, value in env.items()]

    return ["env", "-i", *env_flags, sys.executable, *command]


class ColconPlugin(PluginV2):
    @classmethod
    def get_schema(cls) -> Dict[str, Any]:
        return {
            "$schema": "http://json-schema.org/draft-04/schema#",
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "colcon-packages": {
                    "type": "array",
                    "minitems": 1,
                    "uniqueItems": True,
                    "items": {"type": "string"},
                },
                "colcon-cmake-args": {
                    "type": "array",
                    "minitems": 1,
                    "items": {"type": "string"},
                    "default": [],
                },
                "colcon-catkin-cmake-args": {
                    "type": "array",
                    "minitems": 1,
                    "items": {"type": "string"},
                    "default": [],
                },
                "colcon-ament-cmake-args": {
                    "type": "array",
                    "minitems": 1,
                    "items": {"type": "string"},
                    "default": [],
                },
                "colcon-packages-ignore": {
                    "type": "array",
                    "minitems": 1,
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                },
            },
        }

    def get_build_snaps(self) -> Set[str]:
        return set()

    def get_build_packages(self) -> Set[str]:
        return {
            "python3-colcon-common-extensions",
            "python3-rosdep",
            "python3-wstool",
            "python3-rosinstall",
        }

    def get_build_environment(self) -> Dict[str, str]:
        return {
            "AMENT_PYTHON_EXECUTABLE": "/usr/bin/python3",
            "COLCON_PYTHON_EXECUTABLE": "/usr/bin/python3",
            "ROS_PYTHON_VERSION": "3",
        }

    def _get_colcon_build_command(self) -> str:
        cmd: List[str] = [
            "colcon",
            "build",
            "--merge-install",
            "--install-base",
            "$SNAPCRAFT_PART_INSTALL",
        ]

        if self.options.colcon_packages_ignore:
            cmd.extend(["--packages-ignore", *self.options.colcon_packages_ignore])

        if self.options.colcon_packages:
            cmd.extend(["--packages-select", *self.options.colcon_packages])

        if self.options.colcon_ament_cmake_args:
            cmd.extend(["--ament-cmake-args", *self.options.colcon_ament_cmake_args])

        if self.options.colcon_catkin_cmake_args:
            cmd.extend(["--catkin-cmake-args", *self.options.colcon_catkin_cmake_args])

        # Specify the number of workers
        cmd.extend(["--parallel-workers", "${SNAPCRAFT_PARALLEL_BUILD_COUNT}"])

        return " ".join(cmd)

    def _get_stage_runtime_dependencies_command(self):
        env = dict(
            (key, os.environ[key])
            for key in ["PATH", "SNAP", "SNAP_ARCH", "SNAP_NAME", "SNAP_VERSION"]
            if key in os.environ
        )
        env["LC_ALL"] = "C.UTF-8"
        env["LANG"] = "C.UTF-8"

        return " ".join(
            _get_python_command(
                env,
                [
                    os.path.abspath(__file__),
                    "stage-runtime-dependencies",
                    "--part-install",
                    "$SNAPCRAFT_PART_INSTALL",
                    "--ros-distro",
                    "$ROS_DISTRO",
                ],
            )
        )

    def get_build_commands(self) -> List[str]:
        return [
            ". /opt/ros/$ROS_DISTRO/setup.sh",
            "if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rosdep init; fi",
            "rosdep update --include-eol-distros --rosdistro $ROS_DISTRO",
            "rosdep install --from-paths . --default-yes --ignore-packages-from-source",
            self._get_colcon_build_command(),
            self._get_stage_runtime_dependencies_command(),
        ]


@click.group()
def plugin_cli():
    pass


@plugin_cli.command()
@click.option("--part-install", envvar="SNAPCRAFT_PART_INSTALL", required=True)
@click.option("--ros-distro", envvar="ROS_DISTRO", required=True)
def stage_runtime_dependencies(part_install: str, ros_distro: str):
    # TODO: support python packages (only apt currently supported)
    apt_packages: Set[str] = set()
    rosdep_cmd = shutil.which("rosdep")
    if not rosdep_cmd:
        rosdep_cmd = "/snap/snapcraft/current/usr/bin/rosdep"

    for pkg in catkin_packages.find_packages(".").values():
        for dep in pkg.exec_depends:
            cmd = _get_python_command(
                dict(), [rosdep_cmd, "resolve", dep.name, "--rosdistro", ros_distro]
            )
            try:
                proc = subprocess.run(
                    cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT
                )
            except subprocess.CalledProcessError as error:
                click.echo(f"failed to run {cmd!r}: {error.output}")

            parsed = _parse_rosdep_resolve_dependencies(
                dep, proc.stdout.decode().strip()
            )
            apt_packages |= parsed.pop("apt", set())

            if parsed:
                click.echo(f"unhandled dependencies: {parsed!r}")

    if apt_packages:
        install_path = pathlib.Path(part_install)
        stage_packages_path = install_path.parent / "stage_packages"

        Ubuntu.fetch_stage_packages(
            package_names=sorted(apt_packages),
            base="core20",
            stage_packages_path=stage_packages_path,
        )
        Ubuntu.unpack_stage_packages(
            stage_packages_path=stage_packages_path, install_path=install_path
        )


if __name__ == "__main__":
    plugin_cli()
