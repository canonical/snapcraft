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

"""A base class for ROS plugins."""

import abc
import os
import pathlib
import re
import subprocess
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Set, cast

import catkin_pkg.package
import click
from catkin_pkg import packages as catkin_packages
from craft_parts import plugins
from craft_parts.packages import Repository as Repo
from craft_parts.packages.snaps import _get_parsed_snap
from overrides import overrides

from snapcraft.errors import SnapcraftError


class RosdepError(SnapcraftError):
    """Base class for rosdep errors."""


class RosdepUnexpectedResultError(RosdepError):
    """Error for unexpected rosdep results."""

    def __init__(self, dependency, output):
        super().__init__(
            message="Received unexpected result from rosdep when trying to resolve "
            f"{dependency!r}:\n{output}"
        )


def _parse_rosdep_resolve_dependencies(
    dependency_name: str, output: str
) -> Dict[str, Set[str]]:
    # The output of rosdep follows the pattern:
    #
    #    #apt
    #    package1
    #    package2
    #    #pip
    #    pip-package1
    #    pip-package2
    #
    # Split these out into a dict of dependency type -> dependencies.
    delimiters = re.compile(r"\n|\s")
    lines = delimiters.split(output)
    dependencies: Dict[str, Set[str]] = {}
    dependency_set = None
    for line in lines:
        line = line.strip()  # noqa PLW2901
        if line.startswith("#"):
            key = line.strip("# ")
            dependencies[key] = set()
            dependency_set = dependencies[key]
        elif line:
            if dependency_set is None:
                raise RosdepUnexpectedResultError(dependency_name, output)
            dependency_set.add(line)

    return dependencies


class RosPlugin(plugins.Plugin):
    """Base class for ROS-related plugins. Not intended for use by end users."""

    _MAP_CORE_ROSDISTRO = {"core24": "jazzy"}

    @overrides
    def get_build_snaps(self) -> Set[str]:
        return (
            set(self._options.colcon_ros_build_snaps)  # type: ignore
            if self._options.colcon_ros_build_snaps  # type: ignore
            else set()
        )

    @overrides
    def get_build_packages(self) -> Set[str]:
        base = self._part_info.base
        if base == "core22":
            return {"python3-rosdep", "rospack-tools"}
        return {"python3-rosdep", f"ros-{self._MAP_CORE_ROSDISTRO[base]}-ros2pkg"}

    @overrides
    def get_build_environment(self) -> Dict[str, str]:
        return {"ROS_PYTHON_VERSION": "3"}

    @classmethod
    def get_out_of_source_build(cls) -> bool:
        """Return whether the plugin performs out-of-source-tree builds."""
        return True

    @abc.abstractmethod
    def _get_workspace_activation_commands(self) -> List[str]:
        """Return a list of commands source a ROS workspace.

        The commands returned will be run before doing anything else.
        They will be run in a single shell instance with the rest of
        the build step, so these commands can affect the commands that
        follow.

        snapcraftctl can be used in the script to call out to snapcraft
        specific functionality.
        """

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

    def _get_list_packages_commands(self) -> List[str]:
        """Generate a list of ROS 2 packages available in build snaps.

        The ROS 2 workspaces contained in build snaps are crawled with `rospack`
        to establish the list of all available ROS 2 packages.
        The package names are then resolved with `rosdep` to map their names to debs.
        The list is finally saved in the part's install directory.
        """
        cmd = []

        # Clean up previously established list of packages in build snaps
        cmd.append('rm -f "${CRAFT_PART_INSTALL}/.installed_packages.txt"')
        cmd.append('rm -f "${CRAFT_PART_INSTALL}/.build_snaps.txt"')

        if self._options.colcon_ros_build_snaps:  # type: ignore
            base = self._part_info.base
            if base == "core22":
                rospkg_search_env = "ROS_PACKAGE_PATH"
                rospkg_cmd = "rospack list-names"
            else:
                rospkg_search_env = "AMENT_PREFIX_PATH"
                rospkg_cmd = "ros2 pkg list"

            for ros_build_snap in self._options.colcon_ros_build_snaps:  # type: ignore
                snap_name = _get_parsed_snap(ros_build_snap)[0]
                base_path = f"/snap/{snap_name}/current/opt/ros"
                path_ros_sys = f"{base_path}/${{ROS_DISTRO}}/"
                path_ros_app = f"{base_path}/snap/"
                # ros2 pkg does not crawl sub-folders
                if base == "core22":
                    search_path = base_path
                else:
                    search_path = f"{path_ros_sys}:{path_ros_app}"
                # pylint: disable=line-too-long
                cmd.extend(
                    [
                        # Retrieve the list of all ROS packages available in the build snap
                        f"if [ -d {base_path} ]; then",
                        f"{rospkg_search_env}={search_path} "
                        f'{rospkg_cmd} | (xargs rosdep resolve --rosdistro "${{ROS_DISTRO}}" || echo "") | '
                        'awk "/#apt/{getline;print;}" >> "${CRAFT_PART_INSTALL}/.installed_packages.txt"',
                        "fi",
                        # Retrieve the list of all non-ROS packages available in the build snap
                        f'if [ -d "{path_ros_sys}" ]; then',
                        f'rosdep keys --rosdistro "${{ROS_DISTRO}}" --from-paths "{path_ros_sys}" --ignore-packages-from-source '
                        '| (xargs rosdep resolve --rosdistro "${ROS_DISTRO}" || echo "") | grep -v "#" >> "${CRAFT_PART_INSTALL}"/.installed_packages.txt',
                        "fi",
                        f'if [ -d "{path_ros_app}" ]; then',
                        f'rosdep keys --rosdistro "${{ROS_DISTRO}}" --from-paths "{path_ros_app}" --ignore-packages-from-source '
                        '| (xargs rosdep resolve --rosdistro "${ROS_DISTRO}" || echo "") | grep -v "#" >> "${CRAFT_PART_INSTALL}"/.installed_packages.txt',
                        "fi",
                    ]
                )
                # pylint: enable=line-too-long
            cmd.append("")

        return cmd

    def _get_stage_runtime_dependencies_commands(self) -> List[str]:
        env = {"LANG": "C.UTF-8", "LC_ALL": "C.UTF-8"}

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
                    '"${CRAFT_PART_SRC_WORK}"',
                    "--part-install",
                    '"${CRAFT_PART_INSTALL}"',
                    "--ros-version",
                    '"${ROS_VERSION}"',
                    "--ros-distro",
                    '"${ROS_DISTRO}"',
                    "--target-arch",
                    '"${CRAFT_TARGET_ARCH}"',
                    "--stage-cache-dir",
                    str(self._part_info.cache_dir.resolve()),
                    "--base",
                    f"{self._part_info.base}",
                ]
            )
        ]

    @overrides
    def get_build_commands(self) -> List[str]:
        return (
            [  # noqa S608 (false positive on SQL injection)
                "if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then",
                # Preserve http(s)_proxy env var in root for remote-build proxy since rosdep
                # doesn't support proxy
                # https://github.com/ros-infrastructure/rosdep/issues/271
                "sudo --preserve-env=http_proxy,https_proxy rosdep init; fi",
                'rosdep update --include-eol-distros --rosdistro "${ROS_DISTRO}"',
            ]
            # There are a number of unbound vars, disable flag
            # after saving current state to restore after.
            + [
                'state="$(set +o); set -$-"',
                "set +u",
                "",
            ]
            + self._get_workspace_activation_commands()
            # Restore saved state
            + ['eval "${state}"']
            + self._get_list_packages_commands()
            # pylint: disable=line-too-long
            + [
                'rosdep install --default-yes --ignore-packages-from-source --from-paths "${CRAFT_PART_SRC_WORK}"',
            ]
            # pylint: enable=line-too-long
            + [
                'state="$(set +o); set -$-"',
                "set +u",
                "",
            ]
            + self._get_workspace_activation_commands()
            + ['eval "${state}"']
            + self._get_build_commands()
            + self._get_stage_runtime_dependencies_commands()
        )


@click.group()
def plugin_cli():
    """Define the plugin_cli Click group."""


def get_installed_dependencies(installed_packages_path: str) -> Set[str]:
    """Retrieve recursive apt dependencies of a given package list."""
    if os.path.isfile(installed_packages_path):
        try:
            with open(installed_packages_path, encoding="utf8") as file:
                build_snap_packages = set(file.read().split())
                package_dependencies = set()
                for package in build_snap_packages:
                    cmd = [
                        "apt",
                        "depends",
                        "--recurse",
                        "--no-recommends",
                        "--no-suggests",
                        "--no-conflicts",
                        "--no-breaks",
                        "--no-replaces",
                        "--no-enhances",
                        f"{package}",
                    ]
                    click.echo(f"Running {cmd!r}")
                    try:
                        proc = subprocess.run(
                            cmd,
                            check=True,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            env={"PATH": os.environ["PATH"]},
                        )
                    except subprocess.CalledProcessError as error:
                        click.echo(f"failed to run {cmd!r}: {error.output}")
                    else:
                        apt_dependency_regex = re.compile(r"^\w.*$")
                        for line in proc.stdout.decode().strip().split("\n"):
                            if apt_dependency_regex.match(line):
                                package_dependencies.add(line)

                build_snap_packages.update(package_dependencies)
                click.echo(f"Will not fetch staged packages: {build_snap_packages!r}")
                return build_snap_packages
        except OSError:
            pass
    return set()


@plugin_cli.command()
@click.option("--part-src", envvar="CRAFT_PART_SRC", required=True)
@click.option("--part-install", envvar="CRAFT_PART_INSTALL", required=True)
@click.option("--ros-version", envvar="ROS_VERSION", required=True)
@click.option("--ros-distro", envvar="ROS_DISTRO", required=True)
@click.option("--target-arch", envvar="CRAFT_TARGET_ARCH", required=True)
@click.option("--stage-cache-dir", required=True)
@click.option("--base", required=True)
def stage_runtime_dependencies(  # noqa: PLR0913 (too many arguments)
    part_src: str,
    part_install: str,
    ros_version: str,
    ros_distro: str,
    target_arch: str,
    stage_cache_dir: str,
    base: str,
):  # pylint: disable=too-many-arguments
    """Stage the runtime dependencies of the ROS stack using rosdep."""
    click.echo("Staging runtime dependencies...")
    # @todo: support python packages (only apt currently supported)
    apt_packages: Set[str] = set()

    installed_pkgs = cast(
        Iterable[catkin_pkg.package.Package],
        catkin_packages.find_packages(part_install).values(),
    )
    for pkg in catkin_packages.find_packages(part_src).values():
        pkg = cast(catkin_pkg.package.Package, pkg)  # noqa PLW2901
        # Evaluate the conditions of all dependencies
        pkg.evaluate_conditions(
            {
                "ROS_VERSION": ros_version,
                "ROS_DISTRO": ros_distro,
                "ROS_PYTHON_VERSION": "3",
            }
        )
        # Retrieve only the 'exec_depends' which condition are true
        for dep in (
            exec_dep for exec_dep in pkg.exec_depends if exec_dep.evaluated_condition
        ):
            # No need to resolve this dependency if we know it's local
            if any(p for p in installed_pkgs if p.name == dep.name):
                continue

            cmd = ["rosdep", "resolve", dep.name, "--rosdistro", ros_distro]
            try:
                click.echo(f"Running {cmd!r}")
                proc = subprocess.run(
                    cmd,
                    check=True,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    env={"PATH": os.environ["PATH"]},
                )
            except subprocess.CalledProcessError as error:
                click.echo(f"failed to run {cmd!r}: {error.output}")
                raise RosdepError("rosdep encountered an error") from error

            parsed = _parse_rosdep_resolve_dependencies(
                dep, proc.stdout.decode().strip()
            )
            apt_packages |= parsed.pop("apt", set())

            if parsed:
                click.echo(f"unhandled dependencies: {parsed!r}")

    build_snap_packages = get_installed_dependencies(
        part_install + "/.installed_packages.txt"
    )

    if apt_packages:
        package_names = sorted(apt_packages)
        install_path = pathlib.Path(part_install)
        stage_packages_path = install_path.parent / "stage_packages"

        Repo.configure("snapcraft")

        click.echo(f"Fetching stage packages: {package_names!r}")
        fetched_stage_packages = Repo.fetch_stage_packages(
            cache_dir=Path(stage_cache_dir),
            package_names=package_names,
            arch=target_arch,
            base=base,
            stage_packages_path=stage_packages_path,
            packages_filters=build_snap_packages,  # type: ignore
        )

        click.echo(f"Unpacking stage packages: {fetched_stage_packages!r}")
        Repo.unpack_stage_packages(
            stage_packages_path=stage_packages_path, install_path=install_path
        )


if __name__ == "__main__":
    plugin_cli()
