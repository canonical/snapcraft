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

from snapcraft_legacy.plugins.v2.rust import RustPlugin


class RustPluginTest(TestCase):
    def test_schema(self):
        schema = RustPlugin.get_schema()

        self.assertThat(
            schema,
            Equals(
                {
                    "$schema": "http://json-schema.org/draft-04/schema#",
                    "type": "object",
                    "additionalProperties": False,
                    "properties": {
                        "rust-path": {
                            "type": "array",
                            "minItems": 1,
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
                        "rust-channel": {
                            "type": ["string", "null"],
                            "default": None,
                        },
                        "rust-use-global-lto": {
                            "type": "boolean",
                            "default": False,
                        },
                        "rust-no-default-features": {
                            "type": "boolean",
                            "default": False,
                        },
                    },
                    "required": ["source"],
                }
            ),
        )

    def test_get_build_packages(self):
        plugin = RustPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(
            plugin.get_build_packages(),
            Equals({"curl", "gcc", "git", "pkg-config", "findutils"}),
        )

    def test_get_build_environment(self):
        plugin = RustPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(
            plugin.get_build_environment(),
            Equals({"PATH": "${HOME}/.cargo/bin:${PATH}"}),
        )

    def test_get_build_commands(self):
        class Options:
            rust_channel = "stable"
            rust_path = ["."]
            rust_features = []
            rust_no_default_features = True
            rust_use_global_lto = False

        plugin = RustPlugin(part_name="my-part", options=Options())
        plugin._check_rustup = lambda: True

        self.assertThat(
            plugin.get_build_commands(),
            Equals(
                [
                    "rustup update stable",
                    "rustup default stable",
                    dedent(
                        """\
                    if cargo read-manifest --manifest-path "."/Cargo.toml > /dev/null; then
                        cargo install -f --locked --path "." --root "${SNAPCRAFT_PART_INSTALL}" --no-default-features
                        # remove the installation metadata
                        rm -f "${SNAPCRAFT_PART_INSTALL}"/.crates{.toml,2.json}
                    else
                        # virtual workspace is a bit tricky,
                        # we need to build the whole workspace and then copy the binaries ourselves
                        pushd "."
                        cargo build --workspace --release --no-default-features
                        # install the final binaries
                        find ./target/release -maxdepth 1 -executable -exec install -Dvm755 {} "${SNAPCRAFT_PART_INSTALL}" ';'
                        popd
                    fi"""
                    ),
                ]
            ),
        )

    def test_get_install_command_with_features(self):
        class Options:
            rust_channel = "none"
            rust_path = ["path"]
            rust_features = ["my-feature", "your-feature"]
            rust_no_default_features = False
            rust_use_global_lto = False

        plugin = RustPlugin(part_name="my-part", options=Options())
        plugin._check_rustup = lambda: False

        self.assertThat(
            plugin.get_build_commands(),
            Equals(
                [
                    dedent(
                        """\
                    if cargo read-manifest --manifest-path "path"/Cargo.toml > /dev/null; then
                        cargo install -f --locked --path "path" --root "${SNAPCRAFT_PART_INSTALL}" --features 'my-feature your-feature'
                        # remove the installation metadata
                        rm -f "${SNAPCRAFT_PART_INSTALL}"/.crates{.toml,2.json}
                    else
                        # virtual workspace is a bit tricky,
                        # we need to build the whole workspace and then copy the binaries ourselves
                        pushd "path"
                        cargo build --workspace --release --features 'my-feature your-feature'
                        # install the final binaries
                        find ./target/release -maxdepth 1 -executable -exec install -Dvm755 {} "${SNAPCRAFT_PART_INSTALL}" ';'
                        popd
                    fi"""
                    )
                ]
            ),
        )
