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

import abc
import os
import pathlib
import subprocess
import sys
from typing import Dict, List, Set

import click
from catkin_pkg import packages as catkin_packages

from snapcraft.internal.repo import Repo
from snapcraft.plugins.v1._ros.rosdep import _parse_rosdep_resolve_dependencies
from snapcraft.plugins.v2 import PluginV2


class RosPlugin(PluginV2):
    """Base class for ROS-related plugins. Not intended for use by end users."""

    def get_build_snaps(self) -> Set[str]:
        return set()

    def get_build_packages(self) -> Set[str]:
        return {
            "python3-rosdep",
        }

    def get_build_environment(self) -> Dict[str, str]:
        return {
            "ROS_PYTHON_VERSION": "3",
        }

    @property
    def out_of_source_build(self):
        return True

    def _get_workspace_activation_commands(self) -> List[str]:
        """Return a list of commands source a ROS workspace.

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
            ". /opt/ros/$ROS_DISTRO/setup.sh",
            'eval "$(state)"',
        ]

    @abc.abstractmethod
    def _get_build_commands(self) -> List[str]:
        """Return a list of commands to run during the build step.

        The commands returned will be run after rosdep is used to install
        build dependencies, and before staging runtime dependencies.
        These commands are run in a single shell instance with the rest
        of the build step, so these commands can be affected by the commands
        preceding it, and can affect those that follow.

        snapcraftctl can be used in the script to call out to snapcraft
        specific functionality.
        """

    def _get_stage_runtime_dependencies_commands(self) -> List[str]:
        env = dict(LANG="C.UTF-8", LC_ALL="C.UTF-8")

        for key in [
            "PATH",
            "SNAP",
            "SNAP_ARCH",
            "SNAP_NAME",
            "SNAP_VERSION",
            "http_proxy",
            "https_proxy",
        ]:
            if key in os.environ:
                env[key] = os.environ[key]

        env_flags = [f"{key}={value}" for key, value in env.items()]
        return [
            " ".join(
                [
                    "env",
                    "-i",
                    *env_flags,
                    sys.executable,
                    "-I",
                    os.path.abspath(__file__),
                    "stage-runtime-dependencies",
                    "--part-src",
                    "$SNAPCRAFT_PART_SRC",
                    "--part-install",
                    "$SNAPCRAFT_PART_INSTALL",
                    "--ros-distro",
                    "$ROS_DISTRO",
                    "--target-arch",
                    "$SNAPCRAFT_TARGET_ARCH",
                ]
            )
        ]

    def get_build_commands(self) -> List[str]:
        return (
            self._get_workspace_activation_commands()
            + [
                "if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rosdep init; fi",
                "rosdep update --include-eol-distros --rosdistro $ROS_DISTRO",
                "rosdep install --default-yes --ignore-packages-from-source --from-paths $SNAPCRAFT_PART_SRC",
            ]
            + self._get_build_commands()
            + self._get_stage_runtime_dependencies_commands()
        )


@click.group()
def plugin_cli():
    pass


@plugin_cli.command()
@click.option("--part-src", envvar="SNAPCRAFT_PART_SRC", required=True)
@click.option("--part-install", envvar="SNAPCRAFT_PART_INSTALL", required=True)
@click.option("--ros-distro", envvar="ROS_DISTRO", required=True)
@click.option("--target-arch", envvar="SNAPCRAFT_TARGET_ARCH", required=True)
def stage_runtime_dependencies(
    part_src: str, part_install: str, ros_distro: str, target_arch: str
):
    click.echo("Staging runtime dependencies...")
    # TODO: support python packages (only apt currently supported)
    apt_packages: Set[str] = set()

    for pkg in catkin_packages.find_packages(part_src).values():
        for dep in pkg.exec_depends:
            cmd = ["rosdep", "resolve", dep.name, "--rosdistro", ros_distro]
            try:
                click.echo(f"Running {cmd!r}")
                proc = subprocess.run(
                    cmd,
                    check=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    env=dict(PATH=os.environ["PATH"]),
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
        package_names = sorted(apt_packages)
        install_path = pathlib.Path(part_install)
        stage_packages_path = install_path.parent / "stage_packages"

        click.echo(f"Fetching stage packages: {package_names!r}")
        Repo.fetch_stage_packages(
            package_names=package_names,
            base="core20",
            stage_packages_path=stage_packages_path,
            target_arch=target_arch,
        )

        click.echo(f"Unpacking stage packages: {package_names!r}")
        Repo.unpack_stage_packages(
            stage_packages_path=stage_packages_path, install_path=install_path
        )


if __name__ == "__main__":
    plugin_cli()
