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

from textwrap import dedent

from testtools import TestCase
from testtools.matchers import Equals

from snapcraft.plugins.v2.rust import RustPlugin


class RustPluginTest(TestCase):
    def test_schema(self):
        schema = RustPlugin.get_schema()

        self.assertThat(
            schema,
            Equals(
                {
                    "$schema": "http://json-schema.org/draft-04/schema#",
                    "additionalProperties": False,
                    "properties": {
                        "rust-features": {
                            "default": [],
                            "items": {"type": "string"},
                            "type": "array",
                            "uniqueItems": True,
                        },
                        "rust-path": {
                            "default": ["."],
                            "items": {"type": "string"},
                            "maxItems": 1,
                            "minItems": 1,
                            "type": "array",
                            "uniqueItems": True,
                        },
                    },
                    "required": ["source"],
                    "type": "object",
                }
            ),
        )

    def test_get_build_packages(self):
        plugin = RustPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(plugin.get_build_packages(), Equals({"curl", "gcc", "git"}))

    def test_get_build_environment(self):
        plugin = RustPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(
            plugin.get_build_environment(),
            Equals({"PATH": "${HOME}/.cargo/bin:${PATH}"}),
        )

    def test_get_build_commands(self):
        class Options:
            rust_channel = ""
            rust_path = ["."]
            rust_features = []

        plugin = RustPlugin(part_name="my-part", options=Options())

        self.assertThat(
            plugin.get_build_commands(),
            Equals(
                [
                    dedent(
                        """\
                    if ! command -v rustup 2>/dev/null; then
                        curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --no-modify-path --profile=minimal
                        export PATH="${HOME}/.cargo/bin:${PATH}"
                    fi
                        """
                    ),
                    'cargo install --locked --path . --root "${SNAPCRAFT_PART_INSTALL}" --force',
                ]
            ),
        )

    def test_get_install_command_with_features(self):
        class Options:
            rust_channel = ""
            rust_path = ["path"]
            rust_features = ["my-feature", "your-feature"]

        plugin = RustPlugin(part_name="my-part", options=Options())

        self.assertThat(
            plugin._get_install_command(),
            Equals(
                "cargo install --locked --path path --root \"${SNAPCRAFT_PART_INSTALL}\" --force --features 'my-feature your-feature'"
            ),
        )
