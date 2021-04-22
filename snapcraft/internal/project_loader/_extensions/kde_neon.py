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

from collections import namedtuple
from typing import Any, Dict, Optional, Tuple

from ._extension import Extension


_ExtensionInfo = namedtuple("ExtensionInfo", "cmake_args content provider build_snaps")

_Info = dict(
    core18=_ExtensionInfo(
        cmake_args=None,
        content="kde-frameworks-5-core18-all",
        provider="kde-frameworks-5-core18",
        build_snaps=["kde-frameworks-5-core18-sdk/latest/stable"],
    ),
    core20=_ExtensionInfo(
        cmake_args="-DCMAKE_FIND_ROOT_PATH=/snap/kde-frameworks-5-qt-5-15-core20-sdk/current",
        content="kde-frameworks-5-qt-5-15-core20-all",
        provider="kde-frameworks-5-qt-5-15-core20",
        build_snaps=["kde-frameworks-5-qt-5-15-core20-sdk/latest/candidate"],
    ),
)


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
    - opengl (https://snapcraft.io/docs/opengl-interface)
    - wayland (https://snapcraft.io/docs/wayland-interface)
    - x11 (https://snapcraft.io/docs/x11-interface)
    """

    @staticmethod
    def is_experimental(base: Optional[str]) -> bool:
        # TODO: remove experimental once sdk is on stable
        return base == "core20"

    @staticmethod
    def get_supported_bases() -> Tuple[str, ...]:
        return ("core18", "core20")

    @staticmethod
    def get_supported_confinement() -> Tuple[str, ...]:
        return ("strict", "devmode")

    def __init__(self, *, extension_name: str, yaml_data: Dict[str, Any]) -> None:
        super().__init__(extension_name=extension_name, yaml_data=yaml_data)

        info = _Info[yaml_data["base"]]
        self.root_snippet = {
            "assumes": ["snapd2.43"],  # for 'snapctl is-connected'
            "plugs": {
                "desktop": {"mount-host-font-cache": False},
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
                    "content": info.content,
                    "interface": "content",
                    "default-provider": info.provider,
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

        if info.cmake_args is not None:
            self.part_snippet = {
                "build-environment": [{"SNAPCRAFT_CMAKE_ARGS": info.cmake_args}]
            }

        self.app_snippet = {
            "command-chain": ["snap/command-chain/desktop-launch"],
            "plugs": ["desktop", "desktop-legacy", "opengl", "wayland", "x11"],
        }

        self.parts = {
            "kde-neon-extension": {
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/desktop",
                "source-subdir": "kde-neon",
                "plugin": "make",
                "make-parameters": ["PLATFORM_PLUG=kde-frameworks-5-plug"],
                "build-packages": ["g++"],
                "build-snaps": info.build_snaps,
            }
        }
