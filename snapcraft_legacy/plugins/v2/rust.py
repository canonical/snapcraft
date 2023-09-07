# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020-2023 Canonical Ltd
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

"""A Snapcraft plugin for Rust applications.

This Rust plugin is useful for building Rust based parts.

Rust uses cargo to drive the build.

This plugin uses the common plugin keywords as well as those for "sources".
For more information check the 'plugins' topic for the former and the
'sources' topic for the latter.

Additionally, this plugin uses the following plugin-specific keywords:
    - rust-channel
        (string, default "stable")
        Used to select which Rust channel or version to use.
        It can be one of "stable", "beta", "nightly" or a version number.
        If you don't want this plugin to install Rust toolchain for you,
        you can put "none" for this option.

    - rust-features
        (list of strings)
        Features used to build optional dependencies

    - rust-path
        (list of strings, default [.])
        Build specific crates inside the workspace

    - rust-no-default-features
        (boolean, default False)
        Whether to disable the default features in this crate.
        Equivalent to setting `--no-default-features` on the commandline.

    - rust-use-global-lto
        (boolean, default False)
        Whether to use global LTO.
        This option may significantly impact the build performance but
        reducing the final binary size.
        This will forcibly enable LTO for all the crates you specified,
        regardless of whether you have LTO enabled in the Cargo.toml file
"""

import logging
import subprocess
from textwrap import dedent
from typing import Any, Dict, List, Set

from snapcraft_legacy.plugins.v2 import PluginV2

logger = logging.getLogger(__name__)


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

    def get_build_snaps(self) -> Set[str]:
        return set()

    def get_build_packages(self) -> Set[str]:
        return {"curl", "gcc", "git", "pkg-config", "findutils"}

    def get_build_environment(self) -> Dict[str, str]:
        return {"PATH": "${HOME}/.cargo/bin:${PATH}"}

    def _check_system_rust(self) -> bool:
        """Check if Rust is installed on the system."""
        try:
            rust_version = subprocess.check_output(["rustc", "--version"], text=True)
            cargo_version = subprocess.check_output(["cargo", "--version"], text=True)
            return "rustc" in rust_version and "cargo" in cargo_version
        except (subprocess.CalledProcessError, FileNotFoundError):
            return False

    def _check_rustup(self) -> bool:
        try:
            rustup_version = subprocess.check_output(["rustup", "--version"])
            return "rustup" in rustup_version.decode("utf-8")
        except (subprocess.CalledProcessError, FileNotFoundError):
            return False

    def _get_setup_rustup(self, channel: str) -> List[str]:
        return [
            f"""\
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | \
sh -s -- -y --no-modify-path --profile=minimal --default-toolchain {channel}
"""
        ]

    def _get_install_commands(self) -> List[str]:
        """Return a list of commands to run during the pull step."""
        options = self.options
        if not options.rust_channel and self._check_system_rust():
            logger.info("Rust is installed on the system, skipping rustup")
            return []

        rust_channel = options.rust_channel or "stable"
        if rust_channel == "none":
            return []
        if not self._check_rustup():
            logger.info("Rustup not found, installing it")
            return self._get_setup_rustup(rust_channel)
        logger.info("Switch rustup channel to %s", rust_channel)
        return [
            f"rustup update {rust_channel}",
            f"rustup default {rust_channel}",
        ]

    def get_build_commands(self) -> List[str]:
        options = self.options

        rust_build_cmd: List[str] = []
        config_cmd: List[str] = []

        if options.rust_features:
            features_string = " ".join(options.rust_features)
            config_cmd.extend(["--features", f"'{features_string}'"])

        if options.rust_use_global_lto:
            logger.info("Adding overrides for LTO support")
            config_cmd.extend(
                [
                    "--config 'profile.release.lto = true'",
                    "--config 'profile.release.codegen-units = 1'",
                ]
            )

        if options.rust_no_default_features:
            config_cmd.append("--no-default-features")

        for crate in options.rust_path:
            logger.info("Generating build commands for %s", crate)
            config_cmd_string = " ".join(config_cmd)
            # pylint: disable=line-too-long
            rust_build_cmd_single = dedent(
                f"""\
                if cargo read-manifest --manifest-path "{crate}"/Cargo.toml > /dev/null; then
                    cargo install -f --locked --path "{crate}" --root "${{SNAPCRAFT_PART_INSTALL}}" {config_cmd_string}
                    # remove the installation metadata
                    rm -f "${{SNAPCRAFT_PART_INSTALL}}"/.crates{{.toml,2.json}}
                else
                    # virtual workspace is a bit tricky,
                    # we need to build the whole workspace and then copy the binaries ourselves
                    pushd "{crate}"
                    cargo build --workspace --release {config_cmd_string}
                    # install the final binaries
                    find ./target/release -maxdepth 1 -executable -exec install -Dvm755 {{}} "${{SNAPCRAFT_PART_INSTALL}}" ';'
                    popd
                fi"""
            )
            rust_build_cmd.append(rust_build_cmd_single)
        return self._get_install_commands() + rust_build_cmd
