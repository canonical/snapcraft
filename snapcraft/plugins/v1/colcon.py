# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019,2020 Canonical Ltd
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

"""The colcon plugin is useful for building ROS2 parts.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - colcon-packages:
      (list of strings)
      List of colcon packages to build. If not specified, all packages in the
      workspace will be built. If set to an empty list ([]), no packages will
      be built, which could be useful if you only want ROS debs in the snap.
    - colcon-packages-ignore:
      (list of strings)
      List of colcon packages to ignore. If not specified or set to an empty
      list ([]), no packages will be ignored.
    - colcon-source-space:
      (string)
      The source space containing colcon packages (defaults to 'src').
    - colcon-rosdistro:
      (string)
      The ROS distro to use. Available options are bouncy, crystal, dashing, and
      eloquent, all of which are only compatible with core18 as the base. The default
      value is crystal.
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
"""

import collections
import contextlib
import logging
import os
import pathlib
import re
import shutil
import textwrap
from typing import List

from snapcraft import file_utils
from snapcraft.internal import errors, mangling, os_release, repo
from snapcraft.internal.meta.package_repository import (
    PackageRepository,
    PackageRepositoryApt,
)
from snapcraft.plugins.v1 import PluginV1, _python, _ros

logger = logging.getLogger(__name__)

# Map bases to ROS releases
_ROSDISTRO_TO_BASE_MAP = {
    "bouncy": "core18",
    "crystal": "core18",
    "dashing": "core18",
    "eloquent": "core18",
}

# Snaps can still be built with ROS distros that are end-of-life, but such
# things are not supported. Maintain a list so we can warn about such things.
# This really should be using rosdistro to automatically detect the support
# status, but that's a larger feature than we want to implement at this time.
_EOL_ROSDISTROS = ["bouncy", "crystal"]

# Map bases to Ubuntu releases. Every base in _ROSDISTRO_TO_BASE_MAP needs to be
# specified here.
_BASE_TO_UBUNTU_RELEASE_MAP = {"core18": "bionic"}

_SUPPORTED_DEPENDENCY_TYPES = {"apt", "pip"}


class ColconInvalidSystemDependencyError(errors.SnapcraftError):
    fmt = (
        "Package {dependency!r} isn't a valid system dependency. Did you "
        "forget to add it to colcon-packages? If not, add the Ubuntu package "
        "containing it to stage-packages until you can get it into the rosdep "
        "database."
    )

    def __init__(self, dependency):
        super().__init__(dependency=dependency)


class ColconUnsupportedDependencyTypeError(errors.SnapcraftError):
    fmt = (
        "Package {dependency!r} resolved to an unsupported type of "
        "dependency: {dependency_type!r}."
    )

    def __init__(self, dependency_type, dependency):
        super().__init__(dependency_type=dependency_type, dependency=dependency)


class ColconWorkspaceIsRootError(errors.SnapcraftError):
    fmt = (
        "colcon-source-space cannot be the root of the colcon workspace; use a "
        "subdirectory."
    )


class ColconAptDependencyFetchError(errors.SnapcraftError):
    fmt = "Failed to fetch apt dependencies: {message}"

    def __init__(self, message):
        super().__init__(message=message)


class ColconPackagePathNotFoundError(errors.SnapcraftError):
    fmt = "Failed to find package path: {path!r}"

    def __init__(self, path):
        super().__init__(path=path)


class ColconPluginBaseError(errors.PluginBaseError):
    fmt = (
        "The colcon plugin (used by part {part_name!r}) does not support using base "
        "{base!r} with rosdistro {rosdistro!r}."
    )

    def __init__(self, part_name, base, rosdistro):
        super(errors.PluginBaseError, self).__init__(
            part_name=part_name, base=base, rosdistro=rosdistro
        )


class ColconPlugin(PluginV1):
    @classmethod
    def schema(cls):
        schema = super().schema()

        schema["properties"]["colcon-rosdistro"] = {
            "type": "string",
            "default": "crystal",
            "enum": list(_ROSDISTRO_TO_BASE_MAP.keys()),
        }

        schema["properties"]["colcon-packages"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
        }

        schema["properties"]["colcon-source-space"] = {
            "type": "string",
            "default": "src",
        }

        schema["properties"]["colcon-cmake-args"] = {
            "type": "array",
            "minitems": 1,
            "items": {"type": "string"},
            "default": [],
        }

        schema["properties"]["colcon-catkin-cmake-args"] = {
            "type": "array",
            "minitems": 1,
            "items": {"type": "string"},
            "default": [],
        }

        schema["properties"]["colcon-ament-cmake-args"] = {
            "type": "array",
            "minitems": 1,
            "items": {"type": "string"},
            "default": [],
        }

        schema["properties"]["colcon-packages-ignore"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }

        schema["required"] = ["source"]

        return schema

    @classmethod
    def get_pull_properties(cls):
        # Inform Snapcraft of the properties associated with pulling. If these
        # change in the YAML Snapcraft will consider the pull step dirty.
        return ["colcon-packages", "colcon-source-space", "colcon-rosdistro"]

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return [
            "colcon-cmake-args",
            "colcon-catkin-cmake-args",
            "colcon-ament-cmake-args",
            "colcon-packages-ignore",
        ]

    @classmethod
    def get_required_package_repositories(cls) -> List[PackageRepository]:
        codename = os_release.OsRelease().version_codename()
        return [
            PackageRepositoryApt(
                formats=["deb"],
                components=["main"],
                key_id="C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654",
                url="http://packages.ros.org/ros2/ubuntu",
                suites=[codename],
            )
        ]

    @property
    def _pip(self):
        if not self.__pip:
            self.__pip = _python.Pip(
                python_major_version="3",  # ROS2 uses python3
                part_dir=self.partdir,
                install_dir=self.installdir,
                stage_dir=self.project.stage_dir,
            )

        return self.__pip

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.out_of_source_build = True

        self._rosdistro = options.colcon_rosdistro
        if project._get_build_base() != _ROSDISTRO_TO_BASE_MAP[self._rosdistro]:
            raise ColconPluginBaseError(
                self.name, project._get_build_base(), self._rosdistro
            )

        if self._rosdistro in _EOL_ROSDISTROS:
            logger.warning(
                "The {!r} ROS distro has reached end-of-life and is no longer supported. Use at your own risk.".format(
                    self._rosdistro
                )
            )

        # Beta warning. Remove this comment and warning once plugin is stable.
        logger.warning(
            "The colcon plugin is currently in beta, its API may break. Use at your "
            "own risk."
        )

        self.__pip = None
        self._rosdep_path = os.path.join(self.partdir, "rosdep")
        self._ros_underlay = os.path.join(
            self.installdir, "opt", "ros", self._rosdistro
        )

        # Colcon packages cannot be installed into the same workspace as upstream ROS
        # components, which are Ament. We need to install them alongside and chain the
        # workspaces (upstream is the underlay, stuff we're building here is the
        # overlay).
        self._ros_overlay = os.path.join(self.installdir, "opt", "ros", "snap")

        # Always fetch colcon in order to build the workspace
        self.stage_packages.append("python3-colcon-common-extensions")

        # Get a unique set of packages
        self._packages = None
        if options.colcon_packages is not None:
            self._packages = set(options.colcon_packages)

        # The path created via the `source` key (or a combination of `source`
        # and `source-subdir` keys) needs to point to a valid Colcon workspace
        # containing another subdirectory called the "source space." By
        # default, this is a directory named "src," but it can be remapped via
        # the `source-space` key. It's important that the source space is not
        # the root of the Colcon workspace, since Colcon won't work that way
        # and it'll create a circular link that causes rosdep to hang.
        if self.options.source_subdir:
            self._ros_package_path = os.path.join(
                self.sourcedir,
                self.options.source_subdir,
                self.options.colcon_source_space,
            )
        else:
            self._ros_package_path = os.path.join(
                self.sourcedir, self.options.colcon_source_space
            )

        if os.path.abspath(self.sourcedir) == os.path.abspath(self._ros_package_path):
            raise ColconWorkspaceIsRootError()

        self.stage_packages_path = pathlib.Path(self.partdir) / "colcon_stage_packages"

    def env(self, root):
        """Runtime environment for ROS binaries and services."""

        env = [
            'AMENT_PYTHON_EXECUTABLE="{}"'.format(
                os.path.join(root, "usr", "bin", "python3")
            ),
            'COLCON_PYTHON_EXECUTABLE="{}"'.format(
                os.path.join(root, "usr", "bin", "python3")
            ),
            'SNAP_COLCON_ROOT="{}"'.format(root),
        ]

        # Each of these lines is prepended with an `export` when the environment is
        # actually generated. In order to inject real shell code we have to hack it in
        # by appending it on the end of an item already in the environment.
        # FIXME: There should be a better way to do this. LP: #1792034
        env[-1] = env[-1] + "\n\n" + self._source_setup_sh(root)

        return env

    def pull(self):
        """Copy source into build directory and fetch dependencies.

        Colcon packages can specify their system dependencies in their
        package.xml. In order to support that, the Colcon packages are
        interrogated for their dependencies here. Since `stage-packages` are
        already installed by the time this function is run, the dependencies
        from the package.xml are pulled down explicitly.
        """

        super().pull()

        # Make sure the package path exists before continuing. We only care about doing
        # this if there are actually packages to build, which is indicated both by
        # self._packages being None as well as a non-empty list.
        packages_to_build = self._packages is None or len(self._packages) > 0
        if packages_to_build and not os.path.exists(self._ros_package_path):
            raise ColconPackagePathNotFoundError(self._ros_package_path)

        # Use rosdep for dependency detection and resolution
        rosdep = _ros.rosdep.Rosdep(
            ros_distro=self._rosdistro,
            ros_version="2",
            ros_package_path=self._ros_package_path,
            rosdep_path=self._rosdep_path,
            ubuntu_distro=_BASE_TO_UBUNTU_RELEASE_MAP[self.project._get_build_base()],
            base=self.project._get_build_base(),
            target_arch=self.project._get_stage_packages_target_arch(),
        )
        rosdep.setup()

        self._setup_dependencies(rosdep)

    def _setup_dependencies(self, rosdep):
        # Parse the Colcon packages to pull out their system dependencies
        system_dependencies = _find_system_dependencies(self._packages, rosdep)

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
            raise ColconAptDependencyFetchError(e.message)
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

        with contextlib.suppress(FileNotFoundError):
            shutil.rmtree(self.stage_packages_path)

        # Clean pip packages, if any
        self._pip.clean_packages()

    def _source_setup_sh(self, root):
        underlaydir = os.path.join(root, "opt", "ros", self._rosdistro)
        overlaydir = os.path.join(root, "opt", "ros", "snap")

        source_script = textwrap.dedent(
            """
            # First, source the upstream ROS underlay
            if [ -f "{underlay_setup}" ]; then
                . "{underlay_setup}"
            fi

            # Then source the overlay
            if [ -f "{overlay_setup}" ]; then
                . "{overlay_setup}"
            fi
        """
        ).format(
            underlay_setup=os.path.join(underlaydir, "setup.sh"),
            overlay_setup=os.path.join(overlaydir, "local_setup.sh"),
        )

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

    def build(self):
        """Build Colcon packages.

        This function runs some pre-build steps to prepare the sources for building in
        the Snapcraft environment, builds the packages with colcon, and finally runs
        some post-build clean steps to prepare the newly-minted install to be packaged
        as a .snap.
        """

        super().build()

        logger.info("Preparing to build colcon packages...")
        self._prepare_build()

        logger.info("Building colcon packages...")
        self._build_colcon_packages()

        logger.info("Cleaning up newly installed colcon packages...")
        self._finish_build()

    def _prepare_build(self):
        # Fix all shebangs to use the in-snap python.
        mangling.rewrite_python_shebangs(self.installdir)

        # Rewrite the prefixes to point to the in-part rosdir instead of the system
        self._fix_prefixes()

        # Each Colcon package distributes .cmake files so they can be found via
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
            self._ros_underlay,
            re.compile(r".*Config.cmake$"),
            re.compile(r'"(.*?/.*?)"'),
            _rewrite_paths,
        )

        file_utils.replace_in_file(
            os.path.join(self.installdir, "usr", "lib", "cmake"),
            re.compile(r".*.cmake$"),
            re.compile(r'"(.*?/.*?)"'),
            _rewrite_paths,
        )

    def _finish_build(self):
        # Fix all shebangs to use the in-snap python.
        mangling.rewrite_python_shebangs(self.installdir)

        # We've finished the build, but we need to make sure we turn the cmake
        # files back into something that doesn't include our installdir. This
        # way it's usable from the staging area, and won't clash with the same
        # file coming from other parts.
        pattern = re.compile(r"^{}".format(self.installdir))

        def _new_path(path):
            return pattern.sub("$ENV{SNAPCRAFT_STAGE}", path)

        self._rewrite_cmake_paths(_new_path)

        # Rewrite prefixes for both the underlay and overlay.
        self._fix_prefixes()

        # If pip dependencies were installed, generate a sitecustomize that
        # allows access to them.
        if self._pip.is_setup() and self._pip.list(user=True):
            _python.generate_sitecustomize(
                "3", stage_dir=self.project.stage_dir, install_dir=self.installdir
            )

    def _fix_prefixes(self):
        installdir_pattern = re.compile(r"^{}".format(self.installdir))
        new_prefix = "$SNAP_COLCON_ROOT"

        def _rewrite_prefix(match):
            # Group 1 is the variable definition, group 2 is the path, which we may need
            # to modify.
            path = match.group(3).strip(" \n\t'\"")

            # Bail early if this isn't even a path, or if it's already been rewritten
            if os.path.sep not in path or new_prefix in path:
                return match.group()

            # If the path doesn't start with the installdir, then it needs to point to
            # the underlay given that the upstream ROS packages are expecting to be in
            # /opt/ros/.
            if not path.startswith(self.installdir):
                path = os.path.join(new_prefix, path.lstrip("/"))

            return match.expand(
                '\\1\\2"{}"\\4'.format(installdir_pattern.sub(new_prefix, path))
            )

        # Set the AMENT_CURRENT_PREFIX throughout to the in-snap prefix
        file_utils.replace_in_file(
            self.installdir,
            re.compile(r""),
            re.compile(r"(\${)(AMENT_CURRENT_PREFIX:=)(.*)(})"),
            _rewrite_prefix,
        )

        # Set the COLCON_CURRENT_PREFIX (if it's in the installdir) to the in-snap
        # prefix
        file_utils.replace_in_file(
            self.installdir,
            re.compile(r""),
            re.compile(
                r"()(COLCON_CURRENT_PREFIX=)(['\"].*{}.*)()".format(self.installdir)
            ),
            _rewrite_prefix,
        )

        # Set the _colcon_prefix_sh_COLCON_CURRENT_PREFIX throughout to the in-snap
        # prefix
        file_utils.replace_in_file(
            self.installdir,
            re.compile(r""),
            re.compile(r"()(_colcon_prefix_sh_COLCON_CURRENT_PREFIX=)(.*)()"),
            _rewrite_prefix,
        )

        # Set the _colcon_package_sh_COLCON_CURRENT_PREFIX throughout to the in-snap
        # prefix
        file_utils.replace_in_file(
            self.installdir,
            re.compile(r""),
            re.compile(r"()(_colcon_package_sh_COLCON_CURRENT_PREFIX=)(.*)()"),
            _rewrite_prefix,
        )

        # Set the _colcon_prefix_chain_sh_COLCON_CURRENT_PREFIX throughout to the in-snap
        # prefix
        file_utils.replace_in_file(
            self.installdir,
            re.compile(r""),
            re.compile(r"()(_colcon_prefix_chain_sh_COLCON_CURRENT_PREFIX=)(.*)()"),
            _rewrite_prefix,
        )

        # Set the _colcon_python_executable throughout to use the in-snap python
        file_utils.replace_in_file(
            self.installdir,
            re.compile(r""),
            re.compile(r"()(_colcon_python_executable=)(.*)()"),
            _rewrite_prefix,
        )

    def _build_colcon_packages(self):
        # Nothing to do if no packages were specified
        if self._packages is not None and len(self._packages) == 0:
            return

        colconcmd = ["colcon", "build", "--merge-install"]

        if self._packages:
            # Specify the packages to be built
            colconcmd.append("--packages-select")
            colconcmd.extend(self._packages)

        if self.options.colcon_packages_ignore:
            colconcmd.extend(
                ["--packages-ignore"] + self.options.colcon_packages_ignore
            )

        # Don't clutter the real ROS workspace-- use the Snapcraft build
        # directory
        colconcmd.extend(["--build-base", self.builddir])

        # Account for a non-default source space by always specifying it
        colconcmd.extend(["--base-paths", self._ros_package_path])

        # Specify that the packages should be installed into the overlay
        colconcmd.extend(["--install-base", self._ros_overlay])

        # Specify the number of workers
        colconcmd.append("--parallel-workers={}".format(self.parallel_build_count))

        # All the arguments that follow are meant for CMake
        colconcmd.append("--cmake-args")

        build_type = "Release"
        if "debug" in self.options.build_attributes:
            build_type = "Debug"
        colconcmd.extend(["-DCMAKE_BUILD_TYPE={}".format(build_type)])

        # Finally, add any cmake-args requested from the plugin options
        colconcmd.extend(self.options.colcon_cmake_args)

        if self.options.colcon_catkin_cmake_args:
            colconcmd.extend(
                ["--catkin-cmake-args"] + self.options.colcon_catkin_cmake_args
            )

        if self.options.colcon_ament_cmake_args:
            colconcmd.extend(
                ["--ament-cmake-args"] + self.options.colcon_ament_cmake_args
            )

        self.run(colconcmd)


def _find_system_dependencies(colcon_packages, rosdep):
    """Find system dependencies for a given set of Colcon packages."""

    resolved_dependencies = {}
    dependencies = set()

    logger.info("Determining system dependencies for Colcon packages...")
    if colcon_packages is not None:
        for package in colcon_packages:
            # Query rosdep for the list of dependencies for this package
            dependencies |= rosdep.get_dependencies(package)
    else:
        # Rather than getting dependencies for an explicit list of packages,
        # let's get the dependencies for the entire workspace.
        dependencies |= rosdep.get_dependencies()

    for dependency in dependencies:
        _resolve_package_dependencies(
            colcon_packages, dependency, rosdep, resolved_dependencies
        )

    # We currently have nested dict structure of:
    #    dependency name -> package type -> package names
    #
    # We want to return a flattened dict of package type -> package names.
    flattened_dependencies = collections.defaultdict(set)
    for dependency_types in resolved_dependencies.values():
        for key, value in dependency_types.items():
            flattened_dependencies[key] |= value

    # Finally, return that dict of dependencies
    return flattened_dependencies


def _resolve_package_dependencies(
    colcon_packages, dependency, rosdep, resolved_dependencies
):
    # No need to resolve this dependency if we know it's local, or if
    # we've already resolved it into a system dependency
    if dependency in resolved_dependencies or (
        colcon_packages and dependency in colcon_packages
    ):
        return

    # In this situation, the package depends on something that we
    # weren't instructed to build. It's probably a system dependency,
    # but the developer could have also forgotten to tell us to build
    # it.
    try:
        these_dependencies = rosdep.resolve_dependency(dependency)
    except _ros.rosdep.RosdepDependencyNotResolvedError:
        raise ColconInvalidSystemDependencyError(dependency)

    for key, value in these_dependencies.items():
        if key not in _SUPPORTED_DEPENDENCY_TYPES:
            raise ColconUnsupportedDependencyTypeError(key, dependency)

        resolved_dependencies[dependency] = {key: value}
