# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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

import logging
import os
import pathlib
import re
import shutil
import subprocess
import sys
from typing import Dict, Set

from snapcraft.internal import errors, repo

logger = logging.getLogger(__name__)


class RosdepPackageNotFoundError(errors.SnapcraftError):
    fmt = "rosdep cannot find Catkin package {package!r}"

    def __init__(self, package):
        super().__init__(package=package)


class RosdepDependencyNotResolvedError(errors.SnapcraftError):
    fmt = "rosdep cannot resolve {dependency!r} into a valid dependency"

    def __init__(self, dependency):
        super().__init__(dependency=dependency)


class RosdepUnexpectedResultError(errors.SnapcraftError):
    fmt = (
        "Received unexpected result from rosdep when trying to resolve "
        "{dependency!r}:\n{output}"
    )

    def __init__(self, dependency, output):
        super().__init__(dependency=dependency, output=output)


class RosdepInitializationError(errors.SnapcraftError):
    fmt = "Failed to initialize rosdep: {message}"

    def __init__(self, message):
        super().__init__(message=message)


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
        line = line.strip()
        if line.startswith("#"):
            key = line.strip("# ")
            dependencies[key] = set()
            dependency_set = dependencies[key]
        elif line:
            if dependency_set is None:
                raise RosdepUnexpectedResultError(dependency_name, output)
            else:
                dependency_set.add(line)

    return dependencies


class Rosdep:
    def __init__(
        self,
        *,
        ros_distro,
        ros_version,
        ros_package_path,
        rosdep_path,
        ubuntu_distro,
        base,
        target_arch,
    ):
        self._ros_distro = ros_distro
        self._ros_version = ros_version
        self._ros_package_path = ros_package_path
        self._rosdep_path = rosdep_path
        self._ubuntu_distro = ubuntu_distro
        self._base = base

        self._rosdep_stage_packages_path = (
            pathlib.Path(self._rosdep_path) / "stage_packages"
        )
        self._rosdep_install_path = os.path.join(self._rosdep_path, "install")
        self._rosdep_sources_path = os.path.join(self._rosdep_path, "sources.list.d")
        self._rosdep_cache_path = os.path.join(self._rosdep_path, "cache")
        self._target_arch = target_arch

    def setup(self):
        # Make sure we can run multiple times without error, while leaving the
        # capability to re-initialize, by making sure we clear the sources.
        if os.path.exists(self._rosdep_sources_path):
            shutil.rmtree(self._rosdep_sources_path)

        os.makedirs(self._rosdep_sources_path)
        os.makedirs(self._rosdep_install_path, exist_ok=True)
        os.makedirs(self._rosdep_cache_path, exist_ok=True)
        self._rosdep_stage_packages_path.mkdir(exist_ok=True)

        # rosdep isn't necessarily a dependency of the project, so we'll unpack
        # it off to the side and use it from there.
        logger.info("Installing rosdep...")
        repo.Ubuntu.fetch_stage_packages(
            package_names=["python-rosdep"],
            stage_packages_path=self._rosdep_stage_packages_path,
            base=self._base,
            target_arch=self._target_arch,
        )
        repo.Ubuntu.unpack_stage_packages(
            stage_packages_path=self._rosdep_stage_packages_path,
            install_path=pathlib.Path(self._rosdep_install_path),
        )

        logger.info("Initializing rosdep database...")
        try:
            self._run(["init"])
        except subprocess.CalledProcessError as e:
            output = e.output.decode(sys.getfilesystemencoding()).strip()
            raise RosdepInitializationError(
                "Error initializing rosdep database:\n{}".format(output)
            )

        logger.info("Updating rosdep database...")
        try:
            self._run(["update", "--include-eol-distros"])
        except subprocess.CalledProcessError as e:
            output = e.output.decode(sys.getfilesystemencoding()).strip()
            raise RosdepInitializationError(
                "Error updating rosdep database:\n{}".format(output)
            )

    def get_dependencies(self, package_name=None):
        """Obtain dependencies for a given package, or entire workspace.

        :param str package_name: Package name for which dependences will be
                                 obtained. If not provided, will obtain
                                 dependencies for the entire workspace.
        """
        command = ["keys"]
        if package_name:
            command.append(package_name)
        else:
            # Adding a few flags that will make rosdep search the entire
            # workspace:
            #
            # -a: select all packages in workspace
            # -i: ignore any resolved keys that are satisfied by other packages
            #     in the workspace
            command.append("-a")
            command.append("-i")
        try:
            output = self._run(command).strip()
            if output:
                return set(output.split("\n"))
            else:
                return set()
        except subprocess.CalledProcessError:
            raise RosdepPackageNotFoundError(package_name)

    def resolve_dependency(self, dependency_name):
        try:
            # rosdep needs three pieces of information here:
            #
            # 1) The dependency we're trying to lookup.
            # 2) The rosdistro being used.
            # 3) The version of Ubuntu being used, even if we're running on
            #    something else.
            output = self._run(
                [
                    "resolve",
                    dependency_name,
                    "--rosdistro",
                    self._ros_distro,
                    "--os",
                    "ubuntu:{}".format(self._ubuntu_distro),
                ]
            )
        except subprocess.CalledProcessError:
            raise RosdepDependencyNotResolvedError(dependency_name)

        return _parse_rosdep_resolve_dependencies(dependency_name, output)

    def _run(self, arguments):
        env = os.environ.copy()

        # Use our own private rosdep and its python
        env["PATH"] = os.path.join(self._rosdep_install_path, "usr", "bin")
        env["PYTHONPATH"] = os.path.join(
            self._rosdep_install_path, "usr", "lib", "python2.7", "dist-packages"
        )

        if self._ros_version == "2":
            env["ROS_PYTHON_VERSION"] = "3"
        else:
            env["ROS_PYTHON_VERSION"] = "2"

        # By default, rosdep uses /etc/ros/rosdep to hold its sources list. We
        # don't want that here since we don't want to touch the host machine
        # (not to mention it would require sudo), so we can redirect it via
        # this environment variable
        env["ROSDEP_SOURCE_PATH"] = self._rosdep_sources_path

        # By default, rosdep saves its cache in $HOME/.ros, which we shouldn't
        # access here, so we'll redirect it with this environment variable.
        env["ROS_HOME"] = self._rosdep_cache_path

        env["ROS_VERSION"] = self._ros_version

        # This environment variable tells rosdep which directory to recursively
        # search for packages.
        env["ROS_PACKAGE_PATH"] = self._ros_package_path

        return (
            subprocess.check_output(["rosdep"] + arguments, env=env)
            .decode("utf8")
            .strip()
        )
