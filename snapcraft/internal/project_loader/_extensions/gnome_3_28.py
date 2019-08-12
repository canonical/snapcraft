# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2019 Canonical Ltd
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

# Import types and tell flake8 to ignore the "unused" List.

from typing import Any, Dict, Tuple

from ._extension import Extension


_PLATFORM_SNAP = dict(core18="gnome-3-28-1804")


class Gnome_3_28Extension(Extension):
    """The Gnome extension.

    This extension is to be used by applications that require GTK+.
    """

    @staticmethod
    def get_supported_bases() -> Tuple[str, ...]:
        return ("core18",)

    @staticmethod
    def get_supported_confinement() -> Tuple[str, ...]:
        return ("strict", "devmode")

    def __init__(self, *, extension_name: str, yaml_data: Dict[str, Any]) -> None:
        super().__init__(extension_name=extension_name, yaml_data=yaml_data)

        platform_snap = _PLATFORM_SNAP[yaml_data.get("base")]
        self.root_snippet = {
            "plugs": {
                "gtk-3-themes": {
                    "interface": "content",
                    "target": "$SNAP/data-dir/themes",
                    "default-provider": "gtk-common-themes:gtk-3-themes",
                },
                "icon-themes": {
                    "interface": "content",
                    "target": "$SNAP/data-dir/icons",
                    "default-provider": "gtk-common-themes:icon-themes",
                },
                "sound-themes": {
                    "interface": "content",
                    "target": "$SNAP/data-dir/sounds",
                    "default-provider": "gtk-common-themes:sound-themes",
                },
                platform_snap: {
                    "interface": "content",
                    "target": "$SNAP/gnome-platform",
                    "default-provider": "{snap}:{slot}".format(
                        snap=platform_snap, slot=platform_snap
                    ),
                },
            },
            "environment": {"SNAP_DESKTOP_RUNTIME": "$SNAP/gnome-platform"},
            "layout": {
                "/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0": {
                    "bind": "$SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0"
                },
                "/usr/share/xml/iso-codes": {
                    "bind": "$SNAP/gnome-platform/usr/share/xml/iso-codes"
                },
            },
        }

        self.app_snippet = {
            "command-chain": ["snap/command-chain/desktop-launch"],
            "plugs": ["desktop", "desktop-legacy", "wayland", "x11"],
        }

        self.parts = {
            "gnome-extension": {
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/desktop",
                "source-subdir": "gnome",
                "plugin": "make",
                "build-packages": ["libgtk-3-dev"],
            }
        }
