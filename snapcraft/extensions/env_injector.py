# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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

"""Extension to automatically set environment variables on snaps."""

from typing import Any, Dict, Optional, Tuple

from overrides import overrides

from .extension import Extension, get_extensions_data_dir


class EnvInjectorExtension(Extension):
    """Extension to automatically set environment variables on snaps.

    This extension allows you to transform snap options into environment
    variables

    It configures your application to run a command-chain that transforms the
    snap options into environment variables automatically.

    - To set global environment variables for all applications **inside** the snap:

    .. code-block:: shell
        sudo snap set <snap-name> env.<key>=<value>

    - To set environment variables for a specific application **inside** the snap:

    .. code-block:: shell
        sudo snap set <snap-name> apps.<app-name>.env.<key>=<value>

    - To set environment file inside the snap:

    .. code-block:: shell
        sudo snap set <snap-name> env-file=<path-to-env-file>

    """

    @staticmethod
    @overrides
    def get_supported_bases() -> Tuple[str, ...]:
        return ("core24",)

    @staticmethod
    @overrides
    def get_supported_confinement() -> Tuple[str, ...]:
        return ("strict", "devmode", "classic")

    @staticmethod
    @overrides
    def is_experimental(base: Optional[str]) -> bool:
        return True

    @overrides
    def get_root_snippet(self) -> Dict[str, Any]:
        return {}

    @overrides
    def get_app_snippet(self, *, app_name: str) -> Dict[str, Any]:
        """Return the app snippet to apply."""
        return {
            "command-chain": ["bin/command-chain/env-exporter"],
            "environment": {
                "env_alias": f"{app_name}",
            },
        }

    @overrides
    def get_part_snippet(self, *, plugin_name: str) -> Dict[str, Any]:
        return {"build-environment": [{"SNAPCRAFT_ENV_INJECTOR_EXTENSION": "true"}]}

    @overrides
    def get_parts_snippet(self) -> Dict[str, Any]:
        toolchain = self.get_toolchain()
        if toolchain is None:
            raise ValueError(
                f"Unsupported architecture for env-injector extension: {self.arch}"
            )
        return {
            "env-injector/env-injector": {
                "source": f"{get_extensions_data_dir()}/env-injector",
                "plugin": "nil",
                "build-snaps": [
                    "rustup",
                ],
                "build-packages": [
                    "musl-tools",  # for static linking
                    "upx-ucl",  # for binary compression
                ],
                "override-build": f"""

      rustup default stable
      rustup target add {toolchain}

      cargo build --target {toolchain} --release
      mkdir -p $SNAPCRAFT_PART_INSTALL/bin/command-chain

      cp target/{toolchain}/release/env-exporter $SNAPCRAFT_PART_INSTALL/bin/command-chain

      # compress the binary
      upx --best --lzma target/{toolchain}/release/env-exporter
      cp target/{toolchain}/release/env-exporter $SNAPCRAFT_PART_INSTALL/bin/command-chain/env-exporter-upx

                """,
            }
        }

    def get_toolchain(self):
        """Get the Rust toolchain for the current architecture."""
        # Dictionary mapping architecture names
        toolchain = {
            "amd64": "x86_64-unknown-linux-gnu",
            "arm64": "aarch64-unknown-linux-gnu",
            # 'armhf': 'armv8-unknown-linux-gnueabihf', # Tier 2 toolchain
            # 'riscv64': 'riscv64gc-unknown-linux-gnu', # Tier 2 toolchain
            # 'ppc64el': 'powerpc64-unknown-linux-gnu', # Tier 2 toolchain
            # 's390x': 's390x-unknown-linux-gnu', # Tier 2 toolchain
        }
        return toolchain.get(self.arch)
