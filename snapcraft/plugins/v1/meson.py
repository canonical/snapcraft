# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018,2020 Canonical Ltd
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

"""The meson plugin is useful for building meson based parts.

Meson based projects are projects that have a meson.build that drives the
build.

This plugin always runs 'meson snapbuild' followed by 'ninja' and
'ninja install'.

Additionally, this plugin uses the following plugin-specific keywords:

    - meson-version:
      (string)
      The version of meson to install from PyPI.
      If unspecified, the latest released version of meson will be used.
    - meson-parameters:
      (list of strings)
      Pass the given parameters to the meson command.

"""

import os
import subprocess

from snapcraft.internal import errors
from snapcraft.plugins.v1 import PluginV1


class MesonPlugin(PluginV1):
    @classmethod
    def schema(cls):
        schema = super().schema()
        schema["properties"]["meson-parameters"] = {
            "type": "array",
            "minitems": 1,
            "uniqueItems": True,
            "items": {"type": "string"},
            "default": [],
        }
        schema["properties"]["meson-version"] = {"type": "string", "default": ""}
        schema["required"] = ["source"]

        return schema

    @classmethod
    def get_pull_properties(cls):
        return ["meson-version"]

    @classmethod
    def get_build_properties(cls):
        return ["meson-parameters"]

    def __init__(self, name, options, project):
        super().__init__(name, options, project)

        self._setup_base_tools(project._get_build_base())

        self.snapbuildname = "snapbuild"
        self.mesonbuilddir = os.path.join(self.builddir, self.snapbuildname)

    def _setup_base_tools(self, base):
        self.build_packages.append("python3-pip")
        self.build_packages.append("python3-setuptools")
        self.build_packages.append("python3-wheel")
        self.build_packages.append("ninja-build")

        if base == "core18":
            self.build_packages.append("python3-distutils")

    def pull(self):
        super().pull()

        if self.options.meson_version:
            meson_package = "meson=={}".format(self.options.meson_version)
        else:
            meson_package = "meson"

        try:
            subprocess.check_call(
                ["python3", "-m", "pip", "install", "-U", meson_package]
            )
        except subprocess.CalledProcessError as call_error:
            raise errors.SnapcraftPluginCommandError(
                command=call_error.cmd,
                part_name=self.name,
                exit_code=call_error.returncode,
            ) from call_error

    def build(self):
        super().build()
        self._run_meson()
        self._run_ninja_build_default()
        self._run_ninja_install()

    def _run_meson(self):
        os.makedirs(self.mesonbuilddir, exist_ok=True)
        meson_command = ["meson"]
        if self.options.meson_parameters:
            meson_command.extend(self.options.meson_parameters)
        meson_command.append(self.snapbuildname)
        self.run(meson_command)

    def _run_ninja_build_default(self):
        ninja_command = ["ninja"]
        self.run(ninja_command, cwd=self.mesonbuilddir)

    def _run_ninja_install(self):
        env = os.environ.copy()
        env["DESTDIR"] = self.installdir
        ninja_install_command = ["ninja", "install"]
        self.run(ninja_install_command, env=env, cwd=self.mesonbuilddir)
