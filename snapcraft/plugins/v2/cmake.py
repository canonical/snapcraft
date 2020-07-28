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

"""The cmake plugin is useful for building cmake based parts.

These are projects that have a CMakeLists.txt that drives the build.
The plugin requires a CMakeLists.txt in the root of the source tree.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - cmake-parameters
      (list of strings)
      parameters to pass to the build using the common cmake semantics.
"""

from typing import Any, Dict, List, Set

from snapcraft.plugins.v2 import PluginV2


class CMakePlugin(PluginV2):
    @classmethod
    def get_schema(cls) -> Dict[str, Any]:
        return {
            "$schema": "http://json-schema.org/draft-04/schema#",
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "cmake-parameters": {
                    "type": "array",
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                }
            },
        }

    def get_build_snaps(self) -> Set[str]:
        return set()

    def get_build_packages(self) -> Set[str]:
        return {"gcc", "cmake"}

    def get_build_environment(self) -> Dict[str, str]:
        return {
            "CMAKE_PREFIX_PATH": "${SNAPCRAFT_STAGE}",
        }

    def _get_cmake_configure_command(self) -> str:
        cmd = ["cmake", '"${SNAPCRAFT_PART_SRC_WORK}"'] + self.options.cmake_parameters

        return " ".join(cmd)

    def get_build_commands(self) -> List[str]:
        return [
            self._get_cmake_configure_command(),
            'cmake --build . -- -j"${SNAPCRAFT_PARALLEL_BUILD_COUNT}"',
            'cmake --build . --target install -- DESTDIR="${SNAPCRAFT_PART_INSTALL}"',
        ]
