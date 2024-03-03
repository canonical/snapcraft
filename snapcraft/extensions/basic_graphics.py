# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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

"""Extension that automates the use of mesa-core22 (and eventually later) and related helper utilities."""
from typing import Any, Dict, Optional, Tuple

from overrides import overrides

from .extension import Extension, get_extensions_data_dir

_MESA_SNAP = {"core22": "mesa-core22"}
_GRAPHICS_NAME = {"core22": "graphics-core22"}
_BASE_TO_VER = {"core22": "22"}
_EXT_NAME = "basic-graphics"


class BasicGraphics(Extension):
    """An extension that provides graphics libraries for snaps that don't need to integrate with a full desktop environment.

    This extension is intended to be used with graphical application snaps.
    It defines a single plug "graphics-core{base version number}" which is a content
    interface to the provider snap ("mesa-core{base version number}" by default) that supplies
    the graphics libraries and other important resources.  It also configures apps to use
    the "opengl", "wayland", and "x11" plugs, which are essential for graphical output on
    Ubuntu-supported platforms.
    """

    @staticmethod
    @overrides
    def get_supported_bases() -> Tuple[str, ...]:
        return ("core22",)

    @staticmethod
    @overrides
    def get_supported_confinement() -> Tuple[str, ...]:
        return "strict", "devmode"

    @staticmethod
    @overrides
    def is_experimental(base: Optional[str]) -> bool:
        return False

    @overrides
    def get_root_snippet(self) -> Dict[str, Any]:
        base = self.yaml_data["base"]
        return {
            "plugs": {
                _GRAPHICS_NAME[base]: {
                    "interface": "content",
                    "target": "$SNAP/graphics",
                    "default-provider": _MESA_SNAP[base],
                },
            },
            "layout": {
                "/usr/share/drirc.d": {
                    "bind": "$SNAP/graphics/drirc.d",
                },
                "/usr/share/libdrm": {
                    "bind": "$SNAP/graphics/libdrm",
                },
                "/usr/share/X11/XErrorDB": {
                    "bind-file": "$SNAP/graphics/X11/XErrorDB",
                },
                "/usr/share/X11/locale": {
                    "bind": "$SNAP/graphics/X11/locale",
                },
            },
        }

    @overrides
    def get_app_snippet(self) -> Dict[str, Any]:
        return {
            "command-chain": [
                "bin/graphics-launch",
            ],
            "plugs": ["opengl", "wayland", "x11"],
        }

    @overrides
    def get_part_snippet(self, *, plugin_name: str) -> Dict[str, Any]:
        return {}

    @overrides
    def get_parts_snippet(self) -> Dict[str, Any]:
        base = self.yaml_data["base"]
        return {
            f"{_EXT_NAME}/{_EXT_NAME}": {
                "source": str(get_extensions_data_dir() / _EXT_NAME),
                "plugin": "dump",
                "override-build": "craftctl default\n"
                                  f"sed -i 's/%GRAPHICS%/{_GRAPHICS_NAME[base]}/g' $CRAFT_PART_INSTALL/bin/graphics-launch\n"
                                  f"sed -i 's/%MESA%/{_MESA_SNAP[base]}/g' $CRAFT_PART_INSTALL/bin/graphics-launch",
                "override-prime": "craftctl default\n"
                                  f"graphics-cleanup {_MESA_SNAP[base]}",
                "prime": ["bin/graphics-launch"],
                # This should be the last part to prime, so the clean-up can execute properly
                "after": list(self.yaml_data["parts"].keys()),
            },
        }
