# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
#           2023 Scarlett Moore <sgmoore@kde.org>
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

"""Generic KDE NEON extension to support core22 and onwards."""
import dataclasses
import functools
import re
from typing import Any, Dict, List, Optional, Tuple

from overrides import overrides

from .extension import Extension, get_extensions_data_dir, prepend_to_env

_SDK_SNAP = {"core22": "kde-frameworks-5-102-qt-5-15-8-core22-sd"}
_ExtensionInfo = namedtuple("ExtensionInfo", "cmake_args content provider build_snaps")

_Info = dict(
    core22=_ExtensionInfo(
        cmake_args="-DCMAKE_FIND_ROOT_PATH=/snap/kde-frameworks-5-102-qt-5-15-8-core22-sd/current",
        content="kde-frameworks-5-102-qt-5-15-8-core22-all",
        provider="kde-frameworks-5-102-qt-5-15-8-core22",
        build_snaps=["kde-frameworks-5-102-qt-5-15-8-core22-sd/latest/stable"],
    ),
)

@dataclasses.dataclass
class KDESnaps:
    """A structure of KDE related snaps."""

    sdk: str
    content: str
    builtin: bool = True


class KDENEON(Extension):
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
            "plugs": [
                "desktop",
                "desktop-legacy",
                "opengl",
                "wayland",
                "x11"
            ],
        }

    @functools.cached_property
    def kde_snaps(self) -> KDESnaps:
        """Return the KDE related snaps to use to construct the environment."""
        base = self.yaml_data["base"]
        info = _Info[yaml_data[base]]
        sdk_snap = _SDK_SNAP[base]

        build_snaps: info.build_snaps
        content = info.content
        return KDESnaps(sdk=sdk_snap, content=content, builtin=False)

    @overrides
    def get_root_snippet(self) -> Dict[str, Any]:
        platform_snap = self.kde_snaps.content

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
                info.provider: {
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
            "layout": {"/usr/share/X11": {"symlink": "$SNAP/kf5/usr/share/X11"}},
        }
    @overrides
    def get_part_snippet(self) -> Dict[str, Any]:
        sdk_snap = self.kde_snaps.sdk

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
                            f"$SNAPCRAFT_STAGE/usr/share:/snap/{sdk_snap}/current/usr/share",
                            "/usr/share",
                        ],
                    ),
                },
                {
                    "LD_LIBRARY_PATH": prepend_to_env(
                        "LD_LIBRARY_PATH",
                        [
                            f"/snap/{sdk_snap}/current/lib/$CRAFT_ARCH_TRIPLET",
                            f"/snap/{sdk_snap}/current/usr/lib/$CRAFT_ARCH_TRIPLET",
                            f"/snap/{sdk_snap}/current/usr/lib",
                            f"/snap/{sdk_snap}/current/usr/lib/vala-current",
                            f"/snap/{sdk_snap}/current/usr/lib/$CRAFT_ARCH_TRIPLET/pulseaudio",
                        ],
                    ),
                },
                {
                    "PKG_CONFIG_PATH": prepend_to_env(
                        "PKG_CONFIG_PATH",
                        [
                            f"/snap/{sdk_snap}/current/usr/lib/$CRAFT_ARCH_TRIPLET/pkgconfig",
                            f"/snap/{sdk_snap}/current/usr/lib/pkgconfig",
                            f"/snap/{sdk_snap}/current/usr/share/pkgconfig",
                        ],
                    ),
                },
                {
                    "GETTEXTDATADIRS": prepend_to_env(
                        "GETTEXTDATADIRS",
                        [
                            f"/snap/{sdk_snap}/current/usr/share/gettext-current",
                        ],
                    ),
                },
                {
                    "ACLOCAL_PATH": prepend_to_env(
                        "ACLOCAL_PATH",
                        [
                            f"/snap/{sdk_snap}/current/usr/share/aclocal",
                        ],
                    ),
                },
                {
                    "PYTHONPATH": prepend_to_env(
                        "PYTHONPATH",
                        [
                            f"/snap/{sdk_snap}/current/usr/lib/python3.10",
                            f"/snap/{sdk_snap}/current/usr/lib/python3/dist-packages",
                            f"/snap/{sdk_snap}/current/usr/lib/$CRAFT_ARCH_TRIPLET"
                            "/gobject-introspection",
                        ],
                    ),
                },
                {
                if info.cmake_args is not None:
                    "SNAPCRAFT_CMAKE_ARGS": prepend_to_env(
                        "SNAPCRAFT_CMAKE_ARGS",
                        [info.cmake_args],
                    ),
                },
            ],
        }

    @overrides
    def get_parts_snippet(self) -> Dict[str, Any]:
        return {
            "kde-neon-extension": {
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/desktop",
                "source-subdir": "kde-neon",
                "plugin": "make",
                "make-parameters": [f"PLATFORM_PLUG={info.provider}"],
                "build-packages": ["g++"],
                "build-snaps": info.build_snaps,
            }
        }
