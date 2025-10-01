# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2025 Canonical Ltd.
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

from abc import ABC, abstractmethod
from typing import Any

from typing_extensions import override

from .extension import Extension, get_extensions_data_dir


class DotnetExtensionBase(Extension, ABC):
    """
    An extension that eases the creation of snaps that integrate with
    the .NET content snaps.
    """

    @property
    @abstractmethod
    def runtime_content_snap_name(self) -> str:
        """The name of the .NET runtime content snap the extension will interface with."""

    @property
    @abstractmethod
    def versioned_plugin_name(self) -> str:
        """The name of the versioned .NET extension (such as 'dotnet8')"""

    @property
    def runtime_plug_name(self) -> str:
        """The name of the plug used to connect to the .NET runtime content snap."""
        return f"{self.versioned_plugin_name}-runtime"

    @staticmethod
    @override
    def get_supported_bases() -> tuple[str, ...]:
        return ("core24",)

    @staticmethod
    @override
    def get_supported_confinement() -> tuple[str, ...]:
        return "strict", "devmode"

    @staticmethod
    @override
    def is_experimental(base: str | None) -> bool:
        return True

    @override
    def get_root_snippet(self) -> dict[str, Any]:
        runtime_plugs = {
            self.runtime_plug_name: {
                "interface": "content",
                "default-provider": self.runtime_content_snap_name,
                "content": self.runtime_content_snap_name,
                "target": f"$SNAP/opt/{self.versioned_plugin_name}",
            }
        }

        return {"plugs": runtime_plugs}

    @override
    def get_app_snippet(self, *, app_name: str) -> dict[str, Any]:
        return {
            "command-chain": ["bin/command-chain/launcher.sh"],
            "environment": {
                "DOTNET_EXT_CONTENT_SNAP": self.runtime_content_snap_name,
                "DOTNET_EXT_SNAP_NAME": self.yaml_data["name"],
                "DOTNET_EXT_PLUG_NAME": self.runtime_plug_name,
                "DOTNET_ROOT": f"$SNAP/opt/{self.versioned_plugin_name}/dotnet",
            },
            "plugs": [
                self.runtime_plug_name,
            ],
        }

    @override
    def get_part_snippet(self, *, plugin_name: str) -> dict[str, Any]:
        return {}

    @override
    def get_parts_snippet(self) -> dict[str, Any]:
        parts = {}
        base = self.yaml_data["base"]

        parts[f"{self.versioned_plugin_name}/launcher"] = {
            "plugin": "dump",
            "source": f"{get_extensions_data_dir()}/dotnet",
            "override-build": """
mkdir -p $CRAFT_PART_INSTALL/bin/command-chain
cp launcher.sh $CRAFT_PART_INSTALL/bin/command-chain
""",
            "stage": [
                "bin/command-chain/launcher.sh",
            ],
        }

        match base:
            case "core24":
                libicu_version = "74"
            case _:
                raise AssertionError(f"Unsupported base: {base}")

        parts[f"{self.versioned_plugin_name}/prereqs"] = {
            "plugin": "nil",
            "stage-packages": [
                f"libicu{libicu_version}",
                "libunwind8",
                "libssl3t64",
                "liblttng-ust1t64",
                "libbrotli1",
            ],
        }

        return parts
