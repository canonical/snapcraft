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

import os
import logging
import re
import shutil
import subprocess
import sys

from snapcraft.internal import errors, repo

logger = logging.getLogger(__name__)


class RosdepDependencyNotFoundError(errors.SnapcraftError):
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


class Rosdep:
    def __init__(
        self,
        *,
        ros_distro,
        ros_package_path,
        rosdep_path,
        ubuntu_distro,
        ubuntu_sources,
        project
    ):
        self._ros_distro = ros_distro
        self._ros_package_path = ros_package_path
        self._rosdep_path = rosdep_path
        self._ubuntu_distro = ubuntu_distro
        self._ubuntu_sources = ubuntu_sources
        self._project = project

        self._rosdep_install_path = os.path.join(self._rosdep_path, "install")
        self._rosdep_sources_path = os.path.join(self._rosdep_path, "sources.list.d")
        self._rosdep_cache_path = os.path.join(self._rosdep_path, "cache")

    def setup(self):
        # Make sure we can run multiple times without error, while leaving the
        # capability to re-initialize, by making sure we clear the sources.
        if os.path.exists(self._rosdep_sources_path):
            shutil.rmtree(self._rosdep_sources_path)

        os.makedirs(self._rosdep_sources_path)
        os.makedirs(self._rosdep_install_path, exist_ok=True)
        os.makedirs(self._rosdep_cache_path, exist_ok=True)

        # rosdep isn't necessarily a dependency of the project, so we'll unpack
        # it off to the side and use it from there.
        logger.info("Preparing to fetch rosdep...")
        ubuntu = repo.Ubuntu(
            self._rosdep_path,
            sources=self._ubuntu_sources,
            project_options=self._project,
        )

        logger.info("Fetching rosdep...")
        ubuntu.get(["python-rosdep"])

        logger.info("Installing rosdep...")
        ubuntu.unpack(self._rosdep_install_path)

        logger.info("Initializing rosdep database...")
        try:
            self._run(["init"])
        except subprocess.CalledProcessError as e:
            output = e.output.decode(sys.getfilesystemencoding()).strip()
            raise RuntimeError("Error initializing rosdep database:\n{}".format(output))

        logger.info("Updating rosdep database...")
        try:
            self._run(["update"])
        except subprocess.CalledProcessError as e:
            output = e.output.decode(sys.getfilesystemencoding()).strip()
            raise RuntimeError("Error updating rosdep database:\n{}".format(output))

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
            raise FileNotFoundError(
                'Unable to find Catkin package "{}"'.format(package_name)
            )

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
            raise RosdepDependencyNotFoundError(dependency_name)

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
        dependencies = {}
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

    def _run(self, arguments):
        env = os.environ.copy()

        # Use our own private rosdep and its python
        env["PATH"] = os.path.join(self._rosdep_install_path, "usr", "bin")
        env["PYTHONPATH"] = os.path.join(
            self._rosdep_install_path, "usr", "lib", "python2.7", "dist-packages"
        )

        # By default, rosdep uses /etc/ros/rosdep to hold its sources list. We
        # don't want that here since we don't want to touch the host machine
        # (not to mention it would require sudo), so we can redirect it via
        # this environment variable
        env["ROSDEP_SOURCE_PATH"] = self._rosdep_sources_path

        # By default, rosdep saves its cache in $HOME/.ros, which we shouldn't
        # access here, so we'll redirect it with this environment variable.
        env["ROS_HOME"] = self._rosdep_cache_path

        # This environment variable tells rosdep which directory to recursively
        # search for packages.
        env["ROS_PACKAGE_PATH"] = self._ros_package_path

        return (
            subprocess.check_output(["rosdep"] + arguments, env=env)
            .decode("utf8")
            .strip()
        )
