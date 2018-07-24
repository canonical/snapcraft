# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

"""The catkin_tools build plugin is useful for building ROS parts.

This plugin relies on the catkin plugin as the catkin plugin performs
configuration that allows catkin_tools to run.  This plugin uses the
same keywords and configurations as the catkin plugin, the difference
is the installation of and using catkin_tools to build.
"""

import os
import logging

import snapcraft.plugins.catkin
from snapcraft.plugins.catkin import Compilers

import snapcraft

logger = logging.getLogger(__name__)


class CatkinToolsPlugin(snapcraft.plugins.catkin.CatkinPlugin):
    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.stage_packages.append("python-catkin-tools")

        # Beta Warning
        # Remove this comment and warning once catkin tools plugin is stable.
        logger.warn(
            "The catkin tools plugin is currently in beta, "
            "its API may break. Use at your own risk"
        )

    def _prepare_build(self):
        super()._prepare_build()

        self._clean_catkin()

        self._add_catkin_profile()

        self._configure_catkin_profile()

    def _clean_catkin(self):
        # It's possible that this workspace wasn't initialized to be used with
        # catkin-tools, so initialize it first. Note that this is a noop if it
        # was already initialized.
        self._run_in_bash(["catkin", "init"])

        # Clean to prevent conflicts with externally built packages.
        self._run_in_bash(["catkin", "clean", "-y"])

    def _add_catkin_profile(self):
        # Overwrite the default catkin profile to ensure builds
        # aren't affected by profile changes.
        catkincmd = ["catkin", "profile", "add", "-f", "default"]

        self._run_in_bash(catkincmd)

    def _configure_catkin_profile(self):
        # Use catkin config to set all configurations before running build.
        catkincmd = ["catkin", "config"]

        # Use the newly created default profile.
        catkincmd.extend(["--profile", "default"])

        # Configure catkin tools to use the snapcraft source, build, and
        # install directories.
        catkincmd.extend(["--build-space", self.builddir])

        catkincmd.extend(
            ["--source-space", os.path.join(self.builddir, self.options.source_space)]
        )

        catkincmd.extend(["--install", "--install-space", self.rosdir])

        # Add any cmake-args requested from the plugin options.
        compilers = Compilers(
            self._compilers_path, self.PLUGIN_STAGE_SOURCES, self.project
        )

        catkincmd.append("--cmake-args")
        build_type = "Release"
        if "debug" in self.options.build_attributes:
            build_type = "Debug"

        catkincmd.extend(
            [
                '-DCMAKE_C_FLAGS="$CFLAGS {}"'.format(compilers.cflags),
                '-DCMAKE_CXX_FLAGS="$CPPFLAGS {}"'.format(compilers.cxxflags),
                '-DCMAKE_LD_FLAGS="$LDFLAGS {}"'.format(compilers.ldflags),
                "-DCMAKE_C_COMPILER={}".format(compilers.c_compiler_path),
                "-DCMAKE_CXX_COMPILER={}".format(compilers.cxx_compiler_path),
                "-DCMAKE_BUILD_TYPE={}".format(build_type),
            ]
        )

        catkincmd.extend(self.options.catkin_cmake_args)

        self._run_in_bash(catkincmd)

    def _build_catkin_packages(self):
        # Nothing to do if no packages were specified
        if self.catkin_packages is not None and len(self.catkin_packages) == 0:
            return

        # Call catkin build.
        catkincmd = ["catkin", "build"]

        # Prevent notification that the build is complete.
        catkincmd.append("--no-notify")

        # Use the newly created default profile.
        catkincmd.extend(["--profile", "default"])

        if self.catkin_packages:
            catkincmd.extend(self.catkin_packages)

        compilers = Compilers(
            self._compilers_path, self.PLUGIN_STAGE_SOURCES, self.project
        )

        # This command must run in bash due to a bug in Catkin that causes it
        # to explode if there are spaces in the cmake args (which there are).
        self._run_in_bash(catkincmd, env=compilers.environment)
