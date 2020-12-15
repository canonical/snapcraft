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

_PLATFORM_SNAP = dict(core18="kde-frameworks-5-core18")


class ExtensionImpl(Extension):
    """The KDE Neon extension.

    This extension makes it easy to assemble KDE based applications
    using the Neon stack.

    It configures each application with the following plugs:

    \b
    - Common Icon Themes.
    - Common Sound Themes.
    - The Qt5 and KDE Frameworks runtime libraries and utilities.

    For easier desktop integration, it also configures each application
    entry with these additional plugs:

    \b
    - desktop (https://snapcraft.io/docs/desktop-interface)
    - desktop-legacy (https://snapcraft.io/docs/desktop-legacy-interface)
    - wayland (https://snapcraft.io/docs/wayland-interface)
    - x11 (https://snapcraft.io/docs/x11-interface)
    """

    @staticmethod
    def get_supported_bases() -> Tuple[str, ...]:
        return ("core18",)

    @staticmethod
    def get_supported_confinement() -> Tuple[str, ...]:
        return ("strict", "devmode")

    def __init__(self, *, extension_name: str, yaml_data: Dict[str, Any]) -> None:
        super().__init__(extension_name=extension_name, yaml_data=yaml_data)

        base: str = yaml_data["base"]
        platform_snap = _PLATFORM_SNAP[base]
        self.root_snippet = {
            "plugs": {
                "icon-themes": {
                    "interface": "content",
                    "target": "$SNAP/data-dir/icons",
                    "default-provider": "gtk-common-themes",
                },
                "sound-themes": {
                    "interface": "content",
                    "target": "$SNAP/data-dir/sounds",
                    "default-provider": "gtk-common-themes",
                },
                "kde-frameworks-5-plug": {
                    "content": "kde-frameworks-5-core18-all",
                    "interface": "content",
                    "default-provider": platform_snap,
                    "target": "$SNAP/kf5",
                },
            },
            "environment": {"SNAP_DESKTOP_RUNTIME": "$SNAP/kf5"},
            "hooks": {
                "configure": {
                    "plugs": ["desktop"],
                    "command-chain": ["snap/command-chain/hooks-configure-desktop"],
                }
            },
        }

        self.app_snippet = {
            "command-chain": ["snap/command-chain/desktop-launch"],
            "plugs": ["desktop", "desktop-legacy", "wayland", "x11"],
        }

        self.parts = {
            "kde-neon-extension": {
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/desktop",
                "source-subdir": "kde-neon",
                "plugin": "make",
                "build-packages": ["g++"],
            }
        }
