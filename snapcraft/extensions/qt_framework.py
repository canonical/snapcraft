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

"""Generic QT Framework extension to support core22 and onwards."""
import dataclasses
import functools
from typing import Any, Dict, List, Optional, Tuple

from overrides import overrides

from .extension import Extension, prepend_to_env

_SDK_SNAP = {"core22": "qt-framework-sdk"}

_CONTENT_SNAP = {
    "qt6-6": {"core22": "qt-framework-6-6-core22"},
    "qt6-5": {"core22": "qt-framework-6-5-core22"},
    "qt5-15": {"core22": "qt-framework-5-15-core22"},
}


@dataclasses.dataclass
class ExtensionInfo:
    """Content/SDK build information."""

    cmake_args: list


@dataclasses.dataclass
class QTSnaps:
    """A structure of QT related snaps."""

    sdk: dict
    content: str
    builtin: bool = True


class QTFramework(Extension):
    r"""The QT Framework extension.

    This extension makes it easy to assemble QT based applications.

    It configures each application with the following plugs:

    \b
    - Common Icon Themes.
    - Common Sound Themes.
    - The QT Frameworks runtime libraries and utilities.

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
    def get_app_snippet(self) -> Dict[str, Any]:
        return {
            "command-chain": ["snap/command-chain/desktop-launch"],
            "plugs": ["desktop", "desktop-legacy", "opengl", "wayland", "x11"],
            "environment": {
                "QT_PLUGIN_PATH": "$SNAP/qt-framework/usr/plugins:$SNAP/usr/lib/plugins",
            },
        }

    @functools.cached_property
    def qt_snaps(self) -> QTSnaps:
        """Return the QT related snaps to use to construct the environment."""
        base = self.yaml_data["base"]
        sdk_snap = _SDK_SNAP[base]
        sdk_channel = f"{self.name[2:].replace('-','.')}/stable"

        build_snaps: List[str] = []
        for part in self.yaml_data["parts"].values():
            build_snaps.extend(part.get("build-snaps", []))

        builtin = True

        for snap in build_snaps:
            if sdk_snap == snap.split("/")[0]:
                try:
                    sdk_channel = snap.split("/", 1)[1]
                except IndexError:
                    pass
                builtin = False
                break

        sdk = {"snap": sdk_snap, "channel": sdk_channel}

        # The same except the trailing -sd
        content = _CONTENT_SNAP[self.name][base]

        return QTSnaps(sdk=sdk, content=content, builtin=builtin)

    @functools.cached_property
    def ext_info(self) -> ExtensionInfo:
        """Return the extension info cmake_args, provider, content, build_snaps."""
        cmake_args = [
            f"-DCMAKE_FIND_ROOT_PATH=/snap/{self.qt_snaps.sdk['snap']}/current",
            f"-DCMAKE_PREFIX_PATH=/snap/{self.qt_snaps.sdk['snap']}/current/usr",
            "-DZLIB_INCLUDE_DIR=/lib/x86_64-linux-gnu",
        ]

        return ExtensionInfo(cmake_args=cmake_args)

    @overrides
    def get_root_snippet(self) -> Dict[str, Any]:
        return {
            "assumes": ["snapd2.43"],  # for 'snapctl is-connected'
            "compression": "lzo",
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
                self.qt_snaps.content: {
                    "interface": "content",
                    "default-provider": self.qt_snaps.content,
                    "target": "$SNAP/qt-framework",
                },
            },
            "environment": {"SNAP_DESKTOP_RUNTIME": "$SNAP/qt-framework"},
            "hooks": {
                "configure": {
                    "plugs": ["desktop"],
                    "command-chain": ["snap/command-chain/hooks-configure-fonts"],
                }
            },
            "layout": {
                "/usr/share/X11": {"symlink": "$SNAP/qt-framework/usr/share/X11"}
            },
        }

    @overrides
    def get_part_snippet(self, *, plugin_name: str) -> Dict[str, Any]:
        sdk_snap = self.qt_snaps.sdk["snap"]
        cmake_args = self.ext_info.cmake_args

        return {
            "build-environment": [
                {
                    "PATH": prepend_to_env(
                        "PATH", [f"/snap/{sdk_snap}/current/usr/bin"]
                    ),
                },
                {
                    "XDG_DATA_DIRS": prepend_to_env(
                        "XDG_DATA_DIRS",
                        [
                            f"$CRAFT_STAGE/usr/share:/snap/{sdk_snap}/current/usr/share",
                            "/usr/share",
                        ],
                    ),
                },
                {
                    "SNAPCRAFT_CMAKE_ARGS": prepend_to_env(
                        "SNAPCRAFT_CMAKE_ARGS", cmake_args
                    ),
                },
            ],
            "build-packages": [
                "libgl1-mesa-dev",
                "libpcre2-16-0",
                "libglib2.0-0",
                "libdouble-conversion3",
                "libb2-1",
            ],
            "stage-packages": [
                "libpcre2-16-0",
                "libglib2.0-0",
                "libdouble-conversion3",
                "libb2-1",
            ],
        }

    @overrides
    def get_parts_snippet(self) -> Dict[str, Any]:
        sdk_snap = self.qt_snaps.sdk["snap"]
        sdk_channel = self.qt_snaps.sdk["channel"]

        if self.qt_snaps.builtin:
            return {}

        return {
            f"{self.name}/sdk": {
                "plugin": "nil",
                "build-snaps": [
                    f"{sdk_snap}/{sdk_channel}",
                ],
            },
        }
