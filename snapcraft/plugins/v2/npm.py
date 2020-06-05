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

"""The npm plugin is useful for node based parts that use npm.

The plugin uses npm to install dependencies from `package.json`. It
also sets up binaries defined in `package.json` into the `PATH`.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - npm-node-version
      (string)
      The version of nodejs you want the snap to run on.
      This includes npm, as would be downloaded from https://nodejs.org
"""

import os
import platform
from textwrap import dedent
from typing import Any, Dict, List, Set

from snapcraft.plugins.v2 import PluginV2

_NODE_ARCH_FROM_SNAP_ARCH = {
    "i386": "x86",
    "amd64": "x64",
    "armhf": "armv7l",
    "arm64": "arm64",
    "ppc64el": "ppc64le",
    "s390x": "s390x",
}
_NODE_ARCH_FROM_PLATFORM = {"x86_64": {"32bit": "x86", "64bit": "x64"}}


class NpmPlugin(PluginV2):
    @classmethod
    def get_schema(cls) -> Dict[str, Any]:
        return {
            "$schema": "http://json-schema.org/draft-04/schema#",
            "type": "object",
            "additionalProperties": False,
            "properties": {"npm-node-version": {"type": "string"}},
            "required": ["source", "npm-node-version"],
        }

    @staticmethod
    def _get_architecture() -> str:
        snap_arch = os.getenv("SNAP_ARCH")
        # The first scenario is the general case as snapcraft will be running from the snap.
        if snap_arch is not None:
            node_arch = _NODE_ARCH_FROM_SNAP_ARCH[snap_arch]
        # But there may be times when running from a virtualenv while doing development.
        else:
            node_arch = _NODE_ARCH_FROM_PLATFORM[platform.machine()][
                platform.architecture()[0]
            ]

        return node_arch

    def get_build_snaps(self) -> Set[str]:
        return set()

    def get_build_packages(self) -> Set[str]:
        return {"curl", "gcc"}

    def _get_node_command(self) -> str:
        arch = self._get_architecture()
        version = self.options.npm_node_version

        node_uri = (
            f"https://nodejs.org/dist/v{version}/node-v{version}-linux-{arch}.tar.gz"
        )
        # TODO snapcraftctl for downloading assets.
        return dedent(
            f"""\
        if [ ! -f "${{SNAPCRAFT_PART_INSTALL}}/bin/node" ]; then
            curl -s "{node_uri}" | tar xzf - -C "${{SNAPCRAFT_PART_INSTALL}}/" --strip-components=1
        fi
        """
        )

    def get_build_environment(self) -> Dict[str, str]:
        return dict(PATH="${SNAPCRAFT_PART_INSTALL}/bin:${PATH}")

    def get_build_commands(self) -> List[str]:
        return [
            self._get_node_command(),
            'npm install -g --prefix "${SNAPCRAFT_PART_INSTALL}" $(npm pack . | tail -1)',
        ]
