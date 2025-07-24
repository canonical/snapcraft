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

import dataclasses
import functools
from typing import Any

from overrides import overrides

from .extension import Extension, get_extensions_data_dir

_DOTNET_RUNTIME_PLUG_NAME = "dotnet-runtime"


@dataclasses.dataclass
class DotnetAppDetails:
    """Details about the .NET application being snapped.

    :cvar dotnet_version: The version of .NET the application targets.
    :cvar self_contained: Whether the application is self-contained.
    """

    dotnet_version: str | None = None
    self_contained: bool | None = None

    @property
    def content_snap(self) -> str | None:
        """Return the content snap name if applicable."""
        if self.dotnet_version is not None and not self.self_contained:
            result = f"{self.dotnet_version.split('.')[0]}{self.dotnet_version.split('.')[1] if '.' in self.dotnet_version else '0'}"
            return f"dotnet-runtime-{result}"
        return None


class DotnetExtension(Extension):
    """
    An extension that eases the creation of snaps that integrate with
    the .NET content snaps.
    """

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
        base = self.yaml_data["base"]

        if (
            self.get_application_details.dotnet_version is not None
            and self.get_application_details.self_contained is False
        ):
            match base:
                case "core24":
                    runtime_plugs = {
                        _DOTNET_RUNTIME_PLUG_NAME: {
                            "interface": "content",
                            "default-provider": self.get_application_details.content_snap,
                            "content": self.get_application_details.content_snap,
                            "target": "$SNAP/usr/lib",
                        }
                    }
                case _:
                    raise AssertionError(f"Unsupported base: {base}")

            return {"plugs": runtime_plugs}

        return {}

    @overrides
    def get_app_snippet(self, *, app_name: str) -> dict[str, Any]:
        if self.get_application_details.self_contained is False:
            return {
                "command-chain": ["bin/command-chain/launcher.sh"],
                "environment": {
                    "DOTNET_EXT_CONTENT_SNAP": self.get_application_details.content_snap,
                    "DOTNET_EXT_SNAP_NAME": self.yaml_data["name"],
                    "DOTNET_EXT_PLUG_NAME": _DOTNET_RUNTIME_PLUG_NAME,
                    "DOTNET_ROOT": "$SNAP/usr/lib/dotnet",
                },
                "plugs": [
                    _DOTNET_RUNTIME_PLUG_NAME,
                ],
            }
        return {}

    @overrides
    def get_part_snippet(self, *, plugin_name: str) -> dict[str, Any]:
        return {}

    @overrides
    def get_parts_snippet(self) -> dict[str, Any]:
        parts = {}
        base = self.yaml_data["base"]

        if self.get_application_details.self_contained is False:
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

    @functools.cached_property
    def get_application_details(self) -> DotnetAppDetails:
        """Return details about the .NET application."""
        self_contained = None
        dotnet_version = None

        for part in self.yaml_data.get("parts", {}).values():
            if part.get("plugin") == "dotnet":
                self_contained = part.get("dotnet-self-contained", None)
                dotnet_version = part.get("dotnet-version", None)

        return DotnetAppDetails(
            dotnet_version=dotnet_version, self_contained=self_contained
        )
