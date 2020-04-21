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

"""The go plugin can be used for go projects using go.mod.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - go-channel
      (string, default: latest/stable)
      The Snap Store channel to install go from.

    - go-buildtags
      (list of strings)
      Tags to use during the go build. Default is not to use any build tags.
"""

from typing import Any, Dict, List, Set

from snapcraft.plugins.v2 import PluginV2


class GoPlugin(PluginV2):
    @classmethod
    def get_schema(cls) -> Dict[str, Any]:
        return {
            "$schema": "http://json-schema.org/draft-04/schema#",
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "go-channel": {"type": "string", "default": "latest/stable"},
                "go-buildtags": {
                    "type": "array",
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                },
            },
            "required": ["source"],
        }

    def get_build_snaps(self) -> Set[str]:
        return {f"go/{self.options.go_channel}"}

    def get_build_packages(self) -> Set[str]:
        return {"gcc"}

    def get_build_environment(self) -> Dict[str, str]:
        return {
            "SNAPCRAFT_GO_LDFLAGS": "-ldflags -linkmode=external",
            "CGO_ENABLED": "1",
            "GOBIN": "${SNAPCRAFT_PART_INSTALL}/bin",
        }

    def get_build_commands(self) -> List[str]:
        if self.options.go_buildtags:
            tags = "-tags={}".format(",".join(self.options.go_buildtags))
        else:
            tags = ""

        return [
            "go mod download",
            f'go install -p "${{SNAPCRAFT_PARALLEL_BUILD_COUNT}}" {tags} ${{SNAPCRAFT_GO_LDFLAGS}} ./...',
        ]
