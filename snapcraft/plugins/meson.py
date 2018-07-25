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

"""The meson plugin is useful for building meson based parts.

Meson based projects are projects that have a meson.build that drives the
build.

This plugin always runs 'meson snapbuild' followed by 'ninja' and
'ninja install'.

Additionally, this plugin uses the following plugin-specific keywords:

    - meson-parameters:
      (list of strings)
      Pass the given parameters to the meson command.

"""

import os
import snapcraft


class MesonPlugin(snapcraft.BasePlugin):
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

        return schema

    @classmethod
    def get_build_properties(cls):
        return ["meson-parameters"]

    def __init__(self, name, options, project):
        super().__init__(name, options, project)
        self.snapbuildname = "snapbuild"
        self.mesonbuilddir = os.path.join(self.builddir, self.snapbuildname)
        self.build_packages.append("meson")
        self.build_packages.append("ninja-build")

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
