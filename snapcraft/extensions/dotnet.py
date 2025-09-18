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

from abc import abstractmethod
from typing import Any

from overrides import overrides

from .extension import Extension, get_extensions_data_dir

_DOTNET_RUNTIME_PLUG_NAME = "dotnet-runtime"


class DotnetExtensionBase(Extension):
    """
    An extension that eases the creation of snaps that integrate with
    the .NET content snaps.
    """

    @property
    @abstractmethod
    def runtime_content_snap_name(self) -> str:
        """The name of the .NET runtime content snap."""
        raise NotImplementedError("Subclasses must implement runtime_content_snap_name")

    @staticmethod
    @overrides
    def get_supported_bases() -> tuple[str, ...]:
        return ("core24",)

    @staticmethod
    @overrides
    def get_supported_confinement() -> tuple[str, ...]:
        return "strict", "devmode"

    @staticmethod
    @overrides
    def is_experimental(base: str | None) -> bool:
        return True

    @overrides
    def get_root_snippet(self) -> dict[str, Any]:
        runtime_plugs = {
            _DOTNET_RUNTIME_PLUG_NAME: {
                "interface": "content",
                "default-provider": self.runtime_content_snap_name,
                "content": self.runtime_content_snap_name,
                "target": "$SNAP/usr/lib",
            }
        }

        return {"plugs": runtime_plugs}

    @overrides
    def get_app_snippet(self, *, app_name: str) -> dict[str, Any]:
        return {
            "command-chain": ["bin/command-chain/launcher.sh"],
            "environment": {
                "DOTNET_EXT_CONTENT_SNAP": self.runtime_content_snap_name,
                "DOTNET_EXT_SNAP_NAME": self.yaml_data["name"],
                "DOTNET_EXT_PLUG_NAME": _DOTNET_RUNTIME_PLUG_NAME,
                "DOTNET_ROOT": "$SNAP/usr/lib/dotnet",
            },
            "plugs": [
                _DOTNET_RUNTIME_PLUG_NAME,
            ],
        }

    @overrides
    def get_part_snippet(self, *, plugin_name: str) -> dict[str, Any]:
        return {}

    @overrides
    def get_parts_snippet(self) -> dict[str, Any]:
        parts = {}
        base = self.yaml_data["base"]

        parts["dotnet/launcher"] = {
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

        parts["dotnet/prereqs"] = {
            "plugin": "nil",
            "stage-packages": [
                f"libicu{libicu_version}",
            ],
        }

        return parts
