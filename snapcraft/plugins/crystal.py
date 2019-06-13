# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Manas.Tech
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

"""The Crystal plugin can be used for Crystal projects using `shards`.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - crystal-channel:
      (string, default: latest/stable)
      The Snap Store channel to install Crystal from.
"""

from typing import List

import os
import re
import shutil

import snapcraft
from snapcraft.internal import repo, elf

_CRYSTAL_CHANNEL = "latest/stable"


class CrystalPlugin(snapcraft.BasePlugin):
    @classmethod
    def schema(cls):
        schema = super().schema()

        schema["properties"]["crystal-channel"] = {
            "type": "string",
            "default": _CRYSTAL_CHANNEL,
        }

        return schema

    @classmethod
    def get_pull_properties(cls):
        return ["crystal-channel"]

    @property
    def libc6_libs(self) -> List[str]:
        if self._libc6_libs is None:
            self._libc6_libs = repo.Repo.get_package_libraries("libc6")
        return self._libc6_libs

    def __init__(self, name, options, project):
        super().__init__(name, options, project)

        self._libc6_libs = None

        self.build_snaps.append("crystal/{}".format(self.options.crystal_channel))

        self.build_packages.extend(
            [
                "gcc",
                "pkg-config",
                "libpcre3-dev",
                "libevent-dev",
                "libyaml-dev",
                "libgmp-dev",
                "libxml2-dev",
            ]
        )

    def build(self):
        super().build()

        self.run(["shards", "install", "--production"], self.builddir)
        self.run(["shards", "build", "--production"], self.builddir)

        output_bin = os.path.join(self.builddir, "bin")
        install_bin_path = os.path.join(self.installdir, "bin")

        binary_paths = (
            (b, os.path.join(output_bin, b)) for b in os.listdir(output_bin)
        )
        binaries = (b for b in binary_paths if elf.ElfFile.is_elf(b[1]))

        if not binaries:
            raise errors.SnapcraftEnvironmentError("No binaries were built.")

        os.makedirs(install_bin_path, exist_ok=True)

        for binary_name, binary_path in binaries:
            shutil.copy2(binary_path, os.path.join(install_bin_path, binary_name))

            ldd_output = self.run_output(["ldd", binary_path], self.builddir)
            libs_deps = re.findall(r"(\/.*)\s\(", ldd_output)

            for lib_path in libs_deps:
                if lib_path in self.libc6_libs:
                    continue

                lib_install_path = os.path.join(self.installdir, lib_path[1:])
                os.makedirs(os.path.dirname(lib_install_path), exist_ok=True)
                shutil.copy2(lib_path, lib_install_path)
