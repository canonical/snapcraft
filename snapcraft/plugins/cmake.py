# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2016 Canonical Ltd
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

"""The cmake plugin is useful for building cmake based parts.

These are projects that have a CMakeLists.txt that drives the build.
The plugin requires a CMakeLists.txt in the root of the source tree.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - configflags:
      (list of strings)
      configure flags to pass to the build using the common cmake semantics.
"""

import logging
import os

import snapcraft
from snapcraft.internal import errors


logger = logging.getLogger(name=__name__)


class CMakePlugin(snapcraft.BasePlugin):
    @classmethod
    def schema(cls):
        schema = super().schema()
        schema["properties"]["configflags"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }
        # For backwards compatibility
        schema["properties"]["make-parameters"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }
        schema["required"] = ["source"]

        return schema

    @classmethod
    def get_build_properties(cls):
        # Inform Snapcraft of the properties associated with building. If these
        # change in the YAML Snapcraft will consider the build step dirty.
        return super().get_build_properties() + ["configflags"]

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.build_packages.append("cmake")
        self.out_of_source_build = True

        if project.info.base not in ("core16", "core18"):
            raise errors.PluginBaseError(part_name=self.name, base=project.info.base)

        if options.make_parameters:
            logger.warning("make-paramaters is deprecated, ignoring.")

    def build(self):
        source_subdir = getattr(self.options, "source_subdir", None)
        if source_subdir:
            sourcedir = os.path.join(self.sourcedir, source_subdir)
        else:
            sourcedir = self.sourcedir

        env = self._build_environment()

        self.run(
            ["cmake", sourcedir, "-DCMAKE_INSTALL_PREFIX="] + self.options.configflags,
            env=env,
        )

        # TODO: there is a better way to specify the job count on newer versions of cmake
        # https://github.com/Kitware/CMake/commit/1ab3881ec9e809ac5f6cad5cd84048310b8683e2
        self.run(
            [
                "cmake",
                "--build",
                ".",
                "--",
                "-j{}".format(self.project.parallel_build_count),
            ],
            env=env,
        )

        self.run(["cmake", "--build", ".", "--target", "install"], env=env)

    def _build_environment(self):
        env = os.environ.copy()
        env["DESTDIR"] = self.installdir
        env["CMAKE_PREFIX_PATH"] = "$CMAKE_PREFIX_PATH:{}".format(
            self.project.stage_dir
        )
        env["CMAKE_INCLUDE_PATH"] = "$CMAKE_INCLUDE_PATH:" + ":".join(
            ["{0}/include", "{0}/usr/include", "{0}/include/{1}", "{0}/usr/include/{1}"]
        ).format(self.project.stage_dir, self.project.arch_triplet)
        env["CMAKE_LIBRARY_PATH"] = "$CMAKE_LIBRARY_PATH:" + ":".join(
            ["{0}/lib", "{0}/usr/lib", "{0}/lib/{1}", "{0}/usr/lib/{1}"]
        ).format(self.project.stage_dir, self.project.arch_triplet)

        return env
