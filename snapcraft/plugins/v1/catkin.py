# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2020 Canonical Ltd
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

"""The catkin plugin is useful for building ROS parts.

The rosdistro used depends upon the base of the snap:

  - core: Uses Kinetic
  - core16: Uses Kinetic
  - core18: Uses Melodic

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - catkin-packages:
      (list of strings)
      List of catkin packages to build. If not specified, all packages in the
      workspace will be built. If set to an empty list ([]), no packages will
      be built.
    - source-space:
      (string)
      The source space containing Catkin packages. By default this is 'src'.
    - include-roscore:
      (boolean)
      Whether or not to include roscore with the part. Defaults to true.
    - rosinstall-files:
      (list of strings)
      List of rosinstall files to merge while pulling. Paths are relative to
      the source.
    - recursive-rosinstall:
      (boolean)
      Whether or not to recursively merge/update rosinstall files from fetched
      sources. Will continue until all rosinstall files have been merged.
      Defaults to false.
    - catkin-cmake-args:
      (list of strings)
      Configure flags to pass onto the cmake invocation from catkin.
    - underlay:
      (object)
      Used to inform Snapcraft that this snap isn't standalone, and is actually
      overlaying a workspace from another snap via content sharing. Made up of
      two properties:
      - build-path:
        (string)
        Build-time path to existing workspace to underlay the one being built,
        for example '$SNAPCRAFT_STAGE/opt/ros/kinetic'.
      - run-path:
        (string)
        Run-time path of the underlay workspace (e.g. a subdirectory of the
        content interface's 'target' attribute.)
    - catkin-ros-master-uri:
      (string)
      The URI to ros master setting the env variable ROS_MASTER_URI. Defaults
      to http://localhost:11311.
"""

import contextlib
import glob
import logging
import os
import pathlib
import re
import shlex
import shutil
import subprocess
import tempfile
import textwrap
from typing import TYPE_CHECKING, List, Set

from snapcraft import file_utils, formatting_utils
from snapcraft.internal import common, errors, mangling, os_release, repo
from snapcraft.internal.meta.package_repository import (
    PackageRepository,
    PackageRepositoryApt,
)
from snapcraft.plugins.v1 import PluginV1, _python, _ros

if TYPE_CHECKING:
    from snapcraft.project import Project

logger = logging.getLogger(__name__)

# Map bases to ROS releases
_BASE_TO_ROS_RELEASE_MAP = {"core": "kinetic", "core16": "kinetic", "core18": "melodic"}

# Map bases to Ubuntu releases
_BASE_TO_UBUNTU_RELEASE_MAP = {"core": "xenial", "core16": "xenial", "core18": "bionic"}

_SUPPORTED_DEPENDENCY_TYPES = {"apt", "pip"}


def _parse_cmake_arg(arg: str) -> str:
    # Parse cmake arg string that makes catkin happy.

    # The user can specify a list like:
    # catkin-cmake-args:
    # - -DSOMETHING=FOO
    # - -DCMAKE_C_FLAGS=-Wall -Werror
    # - -DCMAKE_CXX_FLAGS="-Wall -Werror"
    # Catkin can handle strings (1) and (2), but will fail on parsing (3)
    # because of the quotes.  It will end up passing "-Wall -Werror" as a
    # single quoted string to c++.  To work around this, we need to
    # evaluate the string like bash would.  We can do this by using
    # shlex.split() and rejoining the string with spaces.

    # Examples:

    # No quotes.
    # >>> test = '-DCMAKE_C_FLAGS=-Wall -Werror'
    # >>> " ".join(shlex.split(test))
    # '-DCMAKE_C_FLAGS=-Wall -Werror'

    # Double quotes.
    # >>> test2 = '-DCMAKE_CXX_FLAGS="-Wall -Werror"'
    # >>> " ".join(shlex.split(test2))
    # '-DCMAKE_CXX_FLAGS=-Wall -Werror'

    # Single quotes.
    # >>> test3 = "-DCMAKE_CXX_FLAGS='-Wall -Werror'"
    # >>> " ".join(shlex.split(test3))
    # '-DCMAKE_CXX_FLAGS=-Wall -Werror'

    # Nested quotes.
    # >>> test4 = '-DCMAKE_CXX_FLAGS=\"-I\'/some/path with spaces\'\" -Wall -Werror'
    # >>> " ".join(shlex.split(test4))
    # "-DCMAKE_CXX_FLAGS=-I'/some/path with spaces' -Wall -Werror"

    return " ".join(shlex.split(arg))


class CatkinInvalidSystemDependencyError(errors.SnapcraftError):
    fmt = (
        "Package {dependency!r} isn't a valid system dependency. Did you "
        "forget to add it to catkin-packages? If not, add the Ubuntu package "
        "containing it to stage-packages until you can get it into the rosdep "
        "database."
    )

    def __init__(self, dependency):
        super().__init__(dependency=dependency)


class CatkinUnsupportedDependencyTypeError(errors.SnapcraftError):
    fmt = (
        "Package {dependency!r} resolved to an unsupported type of "
        "dependency: {dependency_type!r}."
    )

    def __init__(self, dependency_type, dependency):
        super().__init__(dependency_type=dependency_type, dependency=dependency)


class CatkinWorkspaceIsRootError(errors.SnapcraftError):
    fmt = "source-space cannot be the root of the Catkin workspace; use a subdirectory."


class CatkinCannotResolveRoscoreError(errors.SnapcraftError):
    fmt = "Failed to determine system dependency for roscore."


class CatkinAptDependencyFetchError(errors.SnapcraftError):
    fmt = "Failed to fetch apt dependencies: {message}"

    def __init__(self, message):
        super().__init__(message=message)


class CatkinNoHighestVersionPathError(errors.SnapcraftError):
    fmt = "Failed to determine highest path in {path!r}: nothing found."

    def __init__(self, path):
        super().__init__(path=path)


class CatkinGccVersionError(errors.SnapcraftError):
    fmt = "Failed to determine gcc version: {message}"

    def __init__(self, message):
        super().__init__(message=message)


class CatkinPackagePathNotFoundError(errors.SnapcraftError):
    fmt = "Failed to find package path: {path!r}"

    def __init__(self, path):
        super().__init__(path=path)


class CatkinPlugin(PluginV1):
    @classmethod
    def schema(cls):
        schema = super().schema()
        schema["properties"]["catkin-packages"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
        }
        schema["properties"]["source-space"] = {"type": "string", "default": "src"}

        # The default is true since we expect most Catkin packages to be ROS
        # packages. The only reason one wouldn't want to include ROS in the
        # snap is if library snaps exist, which will still likely be the
        # minority.
        schema["properties"]["include-roscore"] = {"type": "boolean", "default": True}

        schema["properties"]["underlay"] = {
            "type": "object",
            "properties": {
                "build-path": {"type": "string"},
                "run-path": {"type": "string"},
            },
            "required": ["build-path", "run-path"],
        }

        schema["properties"]["rosinstall-files"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }

        schema["properties"]["recursive-rosinstall"] = {
            "type": "boolean",
            "default": False,
        }

        schema["properties"]["catkin-cmake-args"] = {
            "type": "array",
            "minitems": 1,
            "items": {"type": "string"},
            "default": [],
        }

        schema["properties"]["catkin-ros-master-uri"] = {
            "type": "string",
            "default": "http://localhost:11311",
        }

        schema["required"] = ["source"]

        return schema

    @classmethod
    def get_pull_properties(cls):
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the pull step dirty.
        return [
            "catkin-packages",
            "source-space",
            "include-roscore",
            "underlay",
            "rosinstall-files",
            "recursive-rosinstall",
        ]

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return ["catkin-cmake-args"]

    @classmethod
    def get_required_package_repositories(self) -> List[PackageRepository]:
        codename = os_release.OsRelease().version_codename()

        return [
            PackageRepositoryApt(
                formats=["deb"],
                components=["main"],
                key_id="C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654",
                url="http://packages.ros.org/ros/ubuntu/",
                suites=[codename],
            )
        ]

    @property
    def _pip(self):
        if not self.__pip:
            self.__pip = _python.Pip(
                python_major_version="2",  # ROS1 only supports python2
                part_dir=self.partdir,
                install_dir=self.installdir,
                stage_dir=self.project.stage_dir,
            )

        return self.__pip

    def __init__(self, name, options, project):
        super().__init__(name, options, project)

        base = self.project._get_build_base()
        self._rosdistro = _BASE_TO_ROS_RELEASE_MAP[base]

        self.build_packages.extend(["gcc", "g++", "libc6-dev", "make", "python-pip"])
        self.__pip = None

        # roslib is the base requiremet to actually create a workspace with
        # setup.sh and the necessary hooks.
        self.stage_packages.append("ros-{}-roslib".format(self._rosdistro))

        # Get a unique set of packages
        self.catkin_packages = None
        if options.catkin_packages is not None:
            self.catkin_packages = set(options.catkin_packages)
        self.stage_packages_path = pathlib.Path(self.partdir) / "catkin_stage_packages"
        self._rosdep_path = os.path.join(self.partdir, "rosdep")
        self._catkin_path = os.path.join(self.partdir, "catkin")
        self._wstool_path = os.path.join(self.partdir, "wstool")

        # The path created via the `source` key (or a combination of `source`
        # and `source-subdir` keys) needs to point to a valid Catkin workspace
        # containing another subdirectory called the "source space." By
        # default, this is a directory named "src," but it can be remapped via
        # the `source-space` key. It's important that the source space is not
        # the root of the Catkin workspace, since Catkin won't work that way
        # and it'll create a circular link that causes rosdep to hang.
        if self.options.source_subdir:
            self._ros_package_path = os.path.join(
                self.sourcedir, self.options.source_subdir, self.options.source_space
            )
        else:
            self._ros_package_path = os.path.join(
                self.sourcedir, self.options.source_space
            )

        if os.path.abspath(self.sourcedir) == os.path.abspath(self._ros_package_path):
            raise CatkinWorkspaceIsRootError()

    def env(self, root):
        """Runtime environment for ROS binaries and services."""

        paths = common.get_library_paths(root, self.project.arch_triplet)
        ld_library_path = formatting_utils.combine_paths(
            paths, prepend="", separator=":"
        )

        env = [
            # This environment variable tells ROS nodes where to find ROS
            # master. It does not affect ROS master, however-- this is just the
            # URI.
            "ROS_MASTER_URI={}".format(self.options.catkin_ros_master_uri),
            # Various ROS tools (e.g. roscore) keep a cache or a log,
            # and use $ROS_HOME to determine where to put them.
            "ROS_HOME=${SNAP_USER_DATA:-/tmp}/ros",
            # FIXME: LP: #1576411 breaks ROS snaps on the desktop, so we'll
            # temporarily work around that bug by forcing the locale to
            # C.UTF-8.
            "LC_ALL=C.UTF-8",
            # The Snapcraft Core will ensure that we get a good LD_LIBRARY_PATH
            # overall, but it defines it after this function runs. Some ROS
            # tools will cause binaries to be run when we source the setup.sh,
            # below, so we need to have a sensible LD_LIBRARY_PATH before then.
            "LD_LIBRARY_PATH=$LD_LIBRARY_PATH:{}".format(ld_library_path),
        ]

        # There's a chicken and egg problem here, everything run gets an
        # env built, even package installation, so the first runs for these
        # will likely fail.
        try:
            # The ROS packaging system tools (e.g. rospkg, etc.) don't go
            # into the ROS install path (/opt/ros/$distro), so we need the
            # PYTHONPATH to include the dist-packages in /usr/lib as well.
            #
            # Note: Empty segments in PYTHONPATH are interpreted as `.`, thus
            # adding the current working directory to the PYTHONPATH. That is
            # not desired in this situation, so take proper precautions when
            # expanding PYTHONPATH: only add it if it's not empty.
            env.append(
                "PYTHONPATH={}${{PYTHONPATH:+:$PYTHONPATH}}".format(
                    common.get_python2_path(root)
                )
            )
        except errors.SnapcraftEnvironmentError as e:
            logger.debug(e)

        # The setup.sh we source below requires the in-snap python. Here we
        # make sure it's in the PATH before it's run.
        env.append("PATH=$PATH:{}/usr/bin".format(root))

        if self.options.underlay:
            script = textwrap.dedent(
                """
                if [ -f {snapcraft_setup} ]; then
                    . {snapcraft_setup}
                fi
            """
            ).format(snapcraft_setup=os.path.join(self.rosdir, "snapcraft-setup.sh"))
        else:
            script = self._source_setup_sh(root, None)

        # Each of these lines is prepended with an `export` when the
        # environment is actually generated. In order to inject real shell code
        # we have to hack it in by appending it on the end of an item already
        # in the environment. FIXME: There should be a better way to do this.
        # LP: #1792034
        env[-1] = env[-1] + "\n\n" + script

        return env

    def pull(self):
        """Copy source into build directory and fetch dependencies.

        Catkin packages can specify their system dependencies in their
        package.xml. In order to support that, the Catkin packages are
        interrogated for their dependencies here. Since `stage-packages` are
        already installed by the time this function is run, the dependencies
        from the package.xml are pulled down explicitly.
        """

        super().pull()

        # There may be nothing contained within the source but a rosinstall
        # file. We need to use it to flesh out the workspace before continuing
        # with the pull.
        if self.options.rosinstall_files or self.options.recursive_rosinstall:
            wstool = _ros.wstool.Wstool(
                self._ros_package_path,
                self._wstool_path,
                self.project,
                self.project._get_build_base(),
            )
            wstool.setup()

            source_path = self.sourcedir
            if self.options.source_subdir:
                source_path = os.path.join(self.sourcedir, self.options.source_subdir)

            # Recursively handling rosinstall files is a superset of handling
            # individual rosinstall files. If both are specified, the recursive
            # option will cover it.
            if self.options.recursive_rosinstall:
                _recursively_handle_rosinstall_files(wstool, source_path)
            else:
                # The rosinstall files in the YAML are relative to the part's
                # source. However, _handle_rosinstall_files requires absolute
                # paths.
                rosinstall_files = set()
                for rosinstall_file in self.options.rosinstall_files:
                    rosinstall_files.add(os.path.join(source_path, rosinstall_file))

                _handle_rosinstall_files(wstool, rosinstall_files)

        # Make sure the package path exists before continuing. We only care
        # about doing this if there are actually packages to build, which is
        # indicated both by self.catkin_packages being None as well as a
        # non-empty list.
        packages_to_build = (
            self.catkin_packages is None or len(self.catkin_packages) > 0
        )
        if packages_to_build and not os.path.exists(self._ros_package_path):
            raise CatkinPackagePathNotFoundError(self._ros_package_path)

        # Validate the underlay. Note that this validation can't happen in
        # __init__ as the underlay will probably only be valid once a
        # dependency has been staged.
        catkin = None
        underlay_build_path = None
        dependency_workspaces = [self.rosdir]
        if self.options.underlay:
            underlay_build_path = self.options.underlay["build-path"]
        if underlay_build_path:
            if not os.path.isdir(underlay_build_path):
                raise errors.SnapcraftEnvironmentError(
                    "Requested underlay ({!r}) does not point to a valid "
                    "directory".format(underlay_build_path)
                )

            if not os.path.isfile(os.path.join(underlay_build_path, "setup.sh")):
                raise errors.SnapcraftEnvironmentError(
                    "Requested underlay ({!r}) does not contain a "
                    "setup.sh".format(underlay_build_path)
                )

            dependency_workspaces.append(underlay_build_path)
            self._generate_snapcraft_setup_sh(self.installdir, underlay_build_path)

        # Use catkin_find to discover dependencies already in the underlay
        catkin = _Catkin(
            self._rosdistro, dependency_workspaces, self._catkin_path, self.project
        )
        catkin.setup()

        # Use rosdep for dependency detection and resolution
        rosdep = _ros.rosdep.Rosdep(
            ros_distro=self._rosdistro,
            ros_version="1",
            ros_package_path=self._ros_package_path,
            rosdep_path=self._rosdep_path,
            ubuntu_distro=_BASE_TO_UBUNTU_RELEASE_MAP[self.project._get_build_base()],
            base=self.project._get_build_base(),
            target_arch=self.project._get_stage_packages_target_arch(),
        )
        rosdep.setup()

        self._setup_dependencies(rosdep, catkin)

    def _setup_dependencies(self, rosdep, catkin):
        # Parse the Catkin packages to pull out their system dependencies
        system_dependencies = _find_system_dependencies(
            self.catkin_packages, rosdep, catkin
        )

        # If the package requires roscore, resolve it into a system dependency
        # as well.
        if self.options.include_roscore:
            roscore = rosdep.resolve_dependency("ros_core")
            if roscore:
                for dependency_type, dependencies in roscore.items():
                    if dependency_type not in system_dependencies:
                        system_dependencies[dependency_type] = set()
                    system_dependencies[dependency_type] |= dependencies
            else:
                raise CatkinCannotResolveRoscoreError()

        # Pull down and install any apt dependencies that were discovered
        self._setup_apt_dependencies(system_dependencies.get("apt"))

        # Pull down and install any pip dependencies that were discovered
        self._setup_pip_dependencies(system_dependencies.get("pip"))

    def _setup_apt_dependencies(self, apt_dependencies):
        if not apt_dependencies:
            return

        logger.info("Installing apt dependencies...")
        try:
            repo.Ubuntu.fetch_stage_packages(
                package_names=apt_dependencies,
                stage_packages_path=self.stage_packages_path,
                base=self.project._get_build_base(),
                target_arch=self.project._get_stage_packages_target_arch(),
            )
        except repo.errors.PackageNotFoundError as e:
            raise CatkinAptDependencyFetchError(e.message)
        repo.Ubuntu.unpack_stage_packages(
            stage_packages_path=self.stage_packages_path,
            install_path=pathlib.Path(self.installdir),
        )

    def _setup_pip_dependencies(self, pip_dependencies):
        if pip_dependencies:
            self._pip.setup()

            logger.info("Fetching pip dependencies...")
            self._pip.download(pip_dependencies)

            logger.info("Installing pip dependencies...")
            self._pip.install(pip_dependencies)

    def clean_pull(self):
        super().clean_pull()

        # Remove the rosdep path, if any
        with contextlib.suppress(FileNotFoundError):
            shutil.rmtree(self._rosdep_path)

        # Remove the catkin path, if any
        with contextlib.suppress(FileNotFoundError):
            shutil.rmtree(self._catkin_path)

        # Remove the catkin path, if any
        with contextlib.suppress(FileNotFoundError):
            shutil.rmtree(self.stage_packages_path)

        # Clean pip packages, if any
        self._pip.clean_packages()

    def _source_setup_sh(self, root, underlay_path):
        rosdir = os.path.join(root, "opt", "ros", self._rosdistro)
        if underlay_path:
            source_script = textwrap.dedent(
                """
                if [ -f {underlay_setup} ]; then
                    set -- --local
                    _CATKIN_SETUP_DIR={underlay} . {underlay_setup}
                    if [ -f {rosdir_setup} ]; then
                        set -- --local --extend
                        _CATKIN_SETUP_DIR={rosdir} . {rosdir_setup}
                    fi
                fi
            """
            ).format(
                underlay=underlay_path,
                underlay_setup=os.path.join(underlay_path, "setup.sh"),
                rosdir=rosdir,
                rosdir_setup=os.path.join(rosdir, "setup.sh"),
            )
        else:
            source_script = textwrap.dedent(
                """
                if [ -f {rosdir_setup} ]; then
                    set -- --local
                    _CATKIN_SETUP_DIR={rosdir} . {rosdir_setup}
                fi
            """
            ).format(rosdir=rosdir, rosdir_setup=os.path.join(rosdir, "setup.sh"))

        # We need to source ROS's setup.sh at this point. However, it accepts
        # arguments (thus will parse $@), and we really don't want it to, since
        # $@ in this context will be meant for the app being launched
        # (LP: #1660852). So we'll backup all args, source the setup.sh, then
        # restore all args for the wrapper's `exec` line.
        return textwrap.dedent(
            """
            # Shell quote arbitrary string by replacing every occurrence of '
            # with '\\'', then put ' at the beginning and end of the string.
            # Prepare yourself, fun regex ahead.
            quote()
            {{
                for i; do
                    printf %s\\\\n "$i" | sed "s/\'/\'\\\\\\\\\'\'/g;1s/^/\'/;\$s/\$/\' \\\\\\\\/"
                done
                echo " "
            }}

            BACKUP_ARGS=$(quote "$@")
            set --
            {}
            eval "set -- $BACKUP_ARGS"
        """  # noqa: W605
        ).format(
            source_script
        )  # noqa

    def _generate_snapcraft_setup_sh(self, root, underlay_path):
        script = self._source_setup_sh(root, underlay_path)
        os.makedirs(self.rosdir, exist_ok=True)
        with open(os.path.join(self.rosdir, "snapcraft-setup.sh"), "w") as f:
            f.write(script)

    @property
    def rosdir(self):
        return os.path.join(self.installdir, "opt", "ros", self._rosdistro)

    def build(self):
        """Build Catkin packages.

        This function runs some pre-build steps to prepare the sources for
        building in the Snapcraft environment, builds the packages via
        catkin_make_isolated, and finally runs some post-build clean steps
        to prepare the newly-minted install to be packaged as a .snap.
        """

        super().build()

        logger.info("Preparing to build Catkin packages...")
        self._prepare_build()

        logger.info("Building Catkin packages...")
        self._build_catkin_packages()

        logger.info("Cleaning up newly installed Catkin packages...")
        self._finish_build()

    def _prepare_build(self):
        self._use_in_snap_python()

        # Each Catkin package distributes .cmake files so they can be found via
        # find_package(). However, the Ubuntu packages pulled down as
        # dependencies contain .cmake files pointing to system paths (e.g.
        # /usr/lib, /usr/include, etc.). They need to be rewritten to point to
        # the install directory.
        def _new_path(path):
            if not path.startswith(self.installdir):
                # Not using os.path.join here as `path` is absolute.
                return self.installdir + path
            return path

        self._rewrite_cmake_paths(_new_path)

        # Also rewrite any occurrence of $SNAPCRAFT_STAGE to be our install
        # directory (this may be the case if stage-snaps were used).
        file_utils.replace_in_file(
            self.rosdir,
            re.compile(r".*Config.cmake$"),
            re.compile(r"\$ENV{SNAPCRAFT_STAGE}"),
            self.installdir,
        )

    def _rewrite_cmake_paths(self, new_path_callable):
        def _rewrite_paths(match):
            paths = match.group(1).strip().split(";")
            for i, path in enumerate(paths):
                # Offer the opportunity to rewrite this path if it's absolute.
                if os.path.isabs(path):
                    paths[i] = new_path_callable(path)

            return '"' + ";".join(paths) + '"'

        # Looking for any path-like string
        file_utils.replace_in_file(
            self.rosdir,
            re.compile(r".*Config.cmake$"),
            re.compile(r'"(.*?/.*?)"'),
            _rewrite_paths,
        )

    def _finish_build(self):
        self._use_in_snap_python()

        # We've finished the build, but we need to make sure we turn the cmake
        # files back into something that doesn't include our installdir. This
        # way it's usable from the staging area, and won't clash with the same
        # file coming from other parts.
        pattern = re.compile(r"^{}".format(self.installdir))

        def _new_path(path):
            return pattern.sub("$ENV{SNAPCRAFT_STAGE}", path)

        self._rewrite_cmake_paths(_new_path)

        # Replace the CMAKE_PREFIX_PATH in _setup_util.sh
        setup_util_file = os.path.join(self.rosdir, "_setup_util.py")
        if os.path.isfile(setup_util_file):
            with open(setup_util_file, "r+") as f:
                pattern = re.compile(r"CMAKE_PREFIX_PATH = '.*/opt/ros.*")
                replaced = pattern.sub("CMAKE_PREFIX_PATH = []", f.read())
                f.seek(0)
                f.truncate()
                f.write(replaced)

        # Set the _CATKIN_SETUP_DIR in setup.sh to a sensible default, removing
        # our installdir (this way it doesn't clash with a setup.sh coming
        # from another part).
        setup_sh_file = os.path.join(self.rosdir, "setup.sh")
        if os.path.isfile(setup_sh_file):
            with open(setup_sh_file, "r+") as f:
                pattern = re.compile(r"\${_CATKIN_SETUP_DIR:=.*}")
                replaced = pattern.sub(
                    "${{_CATKIN_SETUP_DIR:=$SNAP/opt/ros/{}}}".format(self._rosdistro),
                    f.read(),
                )
                f.seek(0)
                f.truncate()
                f.write(replaced)

        if self.options.underlay:
            underlay_run_path = self.options.underlay["run-path"]
            self._generate_snapcraft_setup_sh("$SNAP", underlay_run_path)

        # If pip dependencies were installed, generate a sitecustomize that
        # allows access to them.
        if self._pip.is_setup() and self._pip.list(user=True):
            _python.generate_sitecustomize(
                "2", stage_dir=self.project.stage_dir, install_dir=self.installdir
            )

    def _use_in_snap_python(self):
        # Fix all shebangs to use the in-snap python.
        mangling.rewrite_python_shebangs(self.installdir)

        # Also replace all the /usr/bin/python calls in etc/catkin/profile.d/
        # files with the in-snap python
        profile_d_path = os.path.join(self.rosdir, "etc", "catkin", "profile.d")
        file_utils.replace_in_file(
            profile_d_path, re.compile(r""), re.compile(r"/usr/bin/python"), r"python"
        )

    def _parse_cmake_args(self):
        args: List[str] = list()
        for arg in self.options.catkin_cmake_args:
            cmake_arg = " ".join(shlex.split(arg))
            args.append(cmake_arg)
        return args

    def _build_catkin_packages(self):
        # Nothing to do if no packages were specified
        if self.catkin_packages is not None and len(self.catkin_packages) == 0:
            return

        catkincmd = ["catkin_make_isolated"]

        # Install the package
        catkincmd.append("--install")

        if self.catkin_packages:
            # Specify the packages to be built
            catkincmd.append("--pkg")
            catkincmd.extend(self.catkin_packages)

        # Don't clutter the real ROS workspace-- use the Snapcraft build
        # directory
        catkincmd.extend(["--directory", self.builddir])

        # Account for a non-default source space by always specifying it
        catkincmd.extend(
            ["--source-space", os.path.join(self.builddir, self.options.source_space)]
        )

        # Specify that the package should be installed along with the rest of
        # the ROS distro.
        catkincmd.extend(["--install-space", self.rosdir])

        # Specify the number of workers
        catkincmd.append("-j{}".format(self.parallel_build_count))

        # All the arguments that follow are meant for CMake
        catkincmd.append("--cmake-args")

        build_type = "Release"
        if "debug" in self.options.build_attributes:
            build_type = "Debug"
        catkincmd.append("-DCMAKE_BUILD_TYPE={}".format(build_type))

        # Finally, add any cmake-args requested from the plugin options
        catkincmd.extend(self._parse_cmake_args())

        self.run(catkincmd)

    def snap_fileset(self):
        """Filter useless files out of the snap.

        - opt/ros/<rosdistro>/.rosinstall points to the part installdir, and
          isn't useful from the snap anyway.
        """

        fileset = super().snap_fileset()
        fileset.append(
            "-{}".format(os.path.join("opt", "ros", self._rosdistro, ".rosinstall"))
        )
        return fileset


def _find_system_dependencies(catkin_packages, rosdep, catkin):
    """Find system dependencies for a given set of Catkin packages."""

    resolved_dependencies = {}
    dependencies = set()

    logger.info("Determining system dependencies for Catkin packages...")
    if catkin_packages is not None:
        for package in catkin_packages:
            # Query rosdep for the list of dependencies for this package
            dependencies |= rosdep.get_dependencies(package)
    else:
        # Rather than getting dependencies for an explicit list of packages,
        # let's get the dependencies for the entire workspace.
        dependencies |= rosdep.get_dependencies()

    for dependency in dependencies:
        _resolve_package_dependencies(
            catkin_packages, dependency, catkin, rosdep, resolved_dependencies
        )

    # We currently have nested dict structure of:
    #    dependency name -> package type -> package names
    #
    # We want to return a flattened dict of package type -> package names.
    flattened_dependencies = {}
    for dependency_types in resolved_dependencies.values():
        for key, value in dependency_types.items():
            if key not in flattened_dependencies:
                flattened_dependencies[key] = set()
            flattened_dependencies[key] |= value

    # Finally, return that dict of dependencies
    return flattened_dependencies


def _resolve_package_dependencies(
    catkin_packages, dependency, catkin, rosdep, resolved_dependencies
):
    # No need to resolve this dependency if we know it's local, or if
    # we've already resolved it into a system dependency
    if dependency in resolved_dependencies or (
        catkin_packages and dependency in catkin_packages
    ):
        return

    if _dependency_is_in_underlay(catkin, dependency):
        # Package was found-- don't pull anything extra to satisfy
        # this dependency.
        logger.debug("Satisfied dependency {!r} in underlay".format(dependency))
        return

    # In this situation, the package depends on something that we
    # weren't instructed to build. It's probably a system dependency,
    # but the developer could have also forgotten to tell us to build
    # it.
    try:
        these_dependencies = rosdep.resolve_dependency(dependency)
    except _ros.rosdep.RosdepDependencyNotResolvedError:
        raise CatkinInvalidSystemDependencyError(dependency)

    for key, value in these_dependencies.items():
        if key not in _SUPPORTED_DEPENDENCY_TYPES:
            raise CatkinUnsupportedDependencyTypeError(key, dependency)

        resolved_dependencies[dependency] = {key: value}


def _dependency_is_in_underlay(catkin, dependency):
    if catkin:
        # Before trying to resolve this dependency into a system
        # dependency, see if it's already in the underlay.
        try:
            catkin.find(dependency)
        except CatkinPackageNotFoundError:
            # No package by that name is available
            pass
        else:
            return True
    return False


def _handle_rosinstall_files(wstool, rosinstall_files):
    """Merge given rosinstall files into our workspace."""

    for rosinstall_file in rosinstall_files:
        logger.info("Merging {}".format(rosinstall_file))
        wstool.merge(rosinstall_file)

    logger.info("Updating workspace...")
    wstool.update()


def _recursively_handle_rosinstall_files(wstool, source_path, *, cache=None):
    "Recursively find and merge rosinstall files and update workspace"

    rosinstall_files: Set[str] = set()
    if not cache:
        cache: Set[str] = set()

    # Walk the entire source directory looking for rosinstall files. Keep track
    # of any we haven't seen previously.
    for root, directories, files in os.walk(source_path):
        for file_name in files:
            path = os.path.join(root, file_name)
            if path.endswith(".rosinstall") and path not in cache:
                rosinstall_files.add(path)

    # If we came across previously-unseen rosinstall files, add them to the
    # cache. Then handle them (merge/update). Finally, walk again. Do this
    # until no new rosinstall files are discovered.
    if rosinstall_files:
        cache.update(rosinstall_files)
        _handle_rosinstall_files(wstool, rosinstall_files)
        _recursively_handle_rosinstall_files(wstool, source_path, cache=cache)


class CatkinPackageNotFoundError(errors.SnapcraftError):
    fmt = "Unable to find Catkin package {package_name!r}"

    def __init__(self, package_name):
        super().__init__(package_name=package_name)


class _Catkin:
    def __init__(
        self,
        ros_distro: str,
        workspaces: List[str],
        catkin_path: str,
        project: "Project",
    ) -> None:
        self._ros_distro = ros_distro
        self._workspaces = workspaces
        self._catkin_path = catkin_path
        self._project = project
        self._catkin_install_path = os.path.join(self._catkin_path, "install")
        self._catkin_stage_packages_path = (
            pathlib.Path(self._catkin_path) / "stage_packages"
        )

    def setup(self):
        os.makedirs(self._catkin_install_path, exist_ok=True)

        # With the introduction of an underlay, we no longer know where Catkin
        # is. Let's just fetch/unpack our own, and use it.
        logger.info("Installing catkin...")
        repo.Ubuntu.fetch_stage_packages(
            package_names=["ros-{}-catkin".format(self._ros_distro)],
            stage_packages_path=self._catkin_stage_packages_path,
            base=self._project._get_build_base(),
            target_arch=self._project._get_stage_packages_target_arch(),
        )
        repo.Ubuntu.unpack_stage_packages(
            stage_packages_path=self._catkin_stage_packages_path,
            install_path=pathlib.Path(self._catkin_install_path),
        )

    def find(self, package_name):
        with contextlib.suppress(subprocess.CalledProcessError):
            path = self._run(["--first-only", package_name]).strip()

            # Not a valid find if the package resolves into our own catkin
            # workspace. That won't be transitioned into the snap.
            if not path.startswith(self._catkin_install_path):
                return path

        raise CatkinPackageNotFoundError(package_name)

    def _run(self, arguments):
        with tempfile.NamedTemporaryFile(mode="w+") as f:
            lines = [
                "export PYTHONPATH={}".format(
                    os.path.join(
                        self._catkin_install_path,
                        "usr",
                        "lib",
                        "python2.7",
                        "dist-packages",
                    )
                )
            ]

            ros_path = os.path.join(
                self._catkin_install_path, "opt", "ros", self._ros_distro
            )
            bin_paths = (
                os.path.join(ros_path, "bin"),
                os.path.join(self._catkin_install_path, "usr", "bin"),
            )
            lines.append(
                "export {}".format(
                    formatting_utils.format_path_variable(
                        "PATH", bin_paths, prepend="", separator=":"
                    )
                )
            )

            # Source our own workspace so we have all of Catkin's dependencies,
            # then source the workspace we're actually supposed to be crawling.
            lines.append(
                "_CATKIN_SETUP_DIR={} source {} --local".format(
                    ros_path, os.path.join(ros_path, "setup.sh")
                )
            )

            for workspace in self._workspaces:
                lines.append(
                    "_CATKIN_SETUP_DIR={} source {} --local --extend".format(
                        workspace, os.path.join(workspace, "setup.sh")
                    )
                )

            lines.append('exec "$@"')
            f.write("\n".join(lines))
            f.flush()
            return (
                subprocess.check_output(
                    ["/bin/bash", f.name, "catkin_find"] + arguments,
                    stderr=subprocess.STDOUT,
                )
                .decode("utf8")
                .strip()
            )


def _get_highest_version_path(path):
    paths = sorted(glob.glob(os.path.join(path, "*")))
    if not paths:
        raise CatkinNoHighestVersionPathError(path)

    return paths[-1]
