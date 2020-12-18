# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

This plugin leverages ninja to build and install.

Additionally, this plugin uses the following plugin-specific keywords:

    - meson-version
      (string)
      The version of meson to install from PyPI.
      If unspecified, the latest released version of meson will be used.

    - meson-parameters
      (list of strings)
      Configure flags to pass to the build using the common meson semantics.
"""

from typing import Any, Dict, List, Set

from snapcraft.plugins.v2 import PluginV2


class MesonPlugin(PluginV2):
    @classmethod
    def get_schema(cls) -> Dict[str, Any]:
        return {
            "$schema": "http://json-schema.org/draft-04/schema#",
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "meson-parameters": {
                    "type": "array",
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                },
                "meson-version": {"type": "string", "default": ""},
            },
            "required": ["source"],
        }

    def get_build_snaps(self) -> Set[str]:
        return set()

    def get_build_packages(self) -> Set[str]:
        return {
            "ninja-build",
            "gcc",
            "python3-pip",
            "python3-setuptools",
            "python3-wheel",
        }

    def get_build_environment(self) -> Dict[str, str]:
        return dict()

    @property
    def out_of_source_build(self):
        return True

    def get_build_commands(self) -> List[str]:
        if self.options.meson_version:
            meson_package = f"meson=={self.options.meson_version}"
        else:
            meson_package = "meson"

        meson_cmd = ["meson"]
        if self.options.meson_parameters:
            meson_cmd.append(" ".join(self.options.meson_parameters))
        meson_cmd.append('"${SNAPCRAFT_PART_SRC_WORK}"')

        return [
            f"/usr/bin/python3 -m pip install -U {meson_package}",
            "[ ! -f build.ninja ] && {}".format(" ".join(meson_cmd)),
            "ninja",
            'DESTDIR="${SNAPCRAFT_PART_INSTALL}" ninja install',
        ]
