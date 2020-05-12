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

"""This rust plugin is useful for building rust based parts.

Rust uses cargo to drive the build.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:

    - rust-features
      (list of strings)
      Features used to build optional dependencies

    - rust-path
      (list of strings, default [.])
      Build specific workspace crates
      Only one item is currently supported.
"""

from textwrap import dedent
from typing import Any, Dict, List, Set

from snapcraft.plugins.v2 import PluginV2


class RustPlugin(PluginV2):
    @classmethod
    def get_schema(cls) -> Dict[str, Any]:
        return {
            "$schema": "http://json-schema.org/draft-04/schema#",
            "type": "object",
            "additionalProperties": False,
            "properties": {
                "rust-path": {
                    "type": "array",
                    "minItems": 1,
                    # TODO support more than one item.
                    "maxItems": 1,
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": ["."],
                },
                "rust-features": {
                    "type": "array",
                    "uniqueItems": True,
                    "items": {"type": "string"},
                    "default": [],
                },
            },
            "required": ["source"],
        }

    def get_build_snaps(self) -> Set[str]:
        return set()

    def get_build_packages(self) -> Set[str]:
        return {"curl", "gcc", "git"}

    def get_build_environment(self) -> Dict[str, str]:
        return {"PATH": "${HOME}/.cargo/bin:${PATH}"}

    def _get_rustup_command(self) -> str:
        return dedent(
            """\
        if ! command -v rustup 2>/dev/null; then
            curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --no-modify-path --profile=minimal
            export PATH="${HOME}/.cargo/bin:${PATH}"
        fi
        """
        )

    def _get_install_command(self) -> str:
        cmd = [
            "cargo",
            "install",
            "--locked",
            "--path",
            self.options.rust_path[0],
            "--root",
            '"${SNAPCRAFT_PART_INSTALL}"',
            "--force",
        ]

        if self.options.rust_features:
            cmd.extend(
                ["--features", "'{}'".format(" ".join(self.options.rust_features))]
            )

        return " ".join(cmd)

    def get_build_commands(self) -> List[str]:
        return [self._get_rustup_command(), self._get_install_command()]
