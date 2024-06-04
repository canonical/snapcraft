# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
#           2023-2024 Scarlett Moore <sgmoore@kde.org>
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

_QT6_SDK_SNAP = {"core22": "kde-qt6-core22-sdk"}
_KF6_SDK_SNAP = {"core22": "kf6-core22-sdk"}


@dataclasses.dataclass
class KDESnaps6:
    """A structure of KDE related snaps."""

    qt6_sdk_snap: str
    kf6_sdk_snap: str
    content_qt6: str
    content_kf6: str
    qt6_builtin: bool = True
    kf6_builtin: bool = True


class KDENeon6(Extension):
    r"""The KDE Neon extension.

    This extension makes it easy to assemble KDE based applications
    using the Neon stack.

    It configures each application with the following plugs:

    \b
    - Common Icon Themes.
    - Common Sound Themes.
    - The Qt6 and KDE Frameworks 6 runtime libraries and utilities.

    For easier desktop integration, it also configures each application
    entry with these additional plugs:

    \b
    - desktop (https://snapcraft.io/docs/desktop-interface)
    - desktop-legacy (https://snapcraft.io/docs/desktop-legacy-interface)
    - opengl (https://snapcraft.io/docs/opengl-interface)
    - wayland (https://snapcraft.io/docs/wayland-interface)
    - x11 (https://snapcraft.io/docs/x11-interface)
    - audio-playback (https://snapcraft.io/docs/audio-playback-interface)
    - unity7 (https://snapcraft.io/docs/unity7-interface)
    - network https://snapcraft.io/docs/network-interface)
    - network-bind (https://snapcraft.io/docs/network-bind-interface)
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
            "command-chain": ["snap/command-chain/desktop-launch6"],
            "plugs": [
                "desktop",
                "desktop-legacy",
                "opengl",
                "wayland",
                "x11",
                "audio-playback",
                "unity7",
                "network",
                "network-bind",
            ],
        }

    @functools.cached_property
    def kde_snaps(self) -> KDESnaps6:
        """Return the KDE related snaps to use to construct the environment."""
        base = self.yaml_data["base"]
        qt6_sdk_snap = _QT6_SDK_SNAP[base]
        kf6_sdk_snap = _KF6_SDK_SNAP[base]

        build_snaps: List[str] = []
        for part in self.yaml_data["parts"].values():
            build_snaps.extend(part.get("build-snaps", []))

        matcher = re.compile(r"kde-qt6-" + base + r"-sdk.*")
        qt6_sdk_snap_candidates = [s for s in build_snaps if matcher.match(s)]
        if qt6_sdk_snap_candidates:
            qt6_sdk_snap = qt6_sdk_snap_candidates[0].split("/")[0]
            qt6_builtin = False
        else:
            qt6_builtin = True

        matcher = re.compile(r"kf6-" + base + r"-sdk.*")
        kf6_sdk_snap_candidates = [s for s in build_snaps if matcher.match(s)]
        if kf6_sdk_snap_candidates:
            kf6_sdk_snap = kf6_sdk_snap_candidates[0].split("/")[0]
            kf6_builtin = False
        else:
            kf6_builtin = True
        # The same except the trailing -sdk
        content_qt6_snap = qt6_sdk_snap[:-4]
        content_kf6_snap = kf6_sdk_snap[:-4]

        return KDESnaps6(
            qt6_sdk_snap=qt6_sdk_snap,
            content_qt6=content_qt6_snap,
            qt6_builtin=qt6_builtin,
            kf6_sdk_snap=kf6_sdk_snap,
            content_kf6=content_kf6_snap,
            kf6_builtin=kf6_builtin,
        )

    @overrides
    def get_root_snippet(self) -> Dict[str, Any]:
        platform_kf6_snap = self.kde_snaps.content_kf6
        content_kf6_snap = self.kde_snaps.content_kf6 + "-all"

        return {
            "assumes": ["snapd2.58.3"],  # for 'snapctl is-connected'
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
                platform_kf6_snap: {
                    "content": content_kf6_snap,
                    "interface": "content",
                    "default-provider": platform_kf6_snap,
                    "target": "$SNAP/kf6",
                },
            },
            "environment": {"SNAP_DESKTOP_RUNTIME": "$SNAP/kf6"},
            "hooks": {
                "configure": {
                    "plugs": ["desktop"],
                    "command-chain": ["snap/command-chain/hooks-configure-desktop"],
                }
            },
            "layout": {
                "/usr/share/X11": {"symlink": "$SNAP/kf6/usr/share/X11"},
                "/usr/share/qt6": {"symlink": "$SNAP/kf6/usr/share/qt6"},
            },
        }

    @overrides
    def get_part_snippet(self, *, plugin_name: str) -> Dict[str, Any]:
        qt6_sdk_snap = self.kde_snaps.qt6_sdk_snap
        kf6_sdk_snap = self.kde_snaps.kf6_sdk_snap

        return {
            "build-environment": [
                {
                    "PATH": prepend_to_env(
                        "PATH",
                        [
                            f"/snap/{qt6_sdk_snap}/current/usr/bin",
                            f"/snap/{kf6_sdk_snap}/current/usr/bin",
                        ],
                    ),
                },
                {
                    "XDG_DATA_DIRS": prepend_to_env(
                        "XDG_DATA_DIRS",
                        [
                            "$CRAFT_STAGE/usr/share",
                            f"/snap/{qt6_sdk_snap}/current/usr/share",
                            f"/snap/{kf6_sdk_snap}/current/usr/share",
                            "/usr/share",
                        ],
                    ),
                },
                {
                    "XDG_CONFIG_HOME": prepend_to_env(
                        "XDG_CONFIG_HOME",
                        [
                            "$CRAFT_STAGE/etc/xdg",
                            f"/snap/{qt6_sdk_snap}/current/etc/xdg",
                            f"/snap/{kf6_sdk_snap}/current/etc/xdg",
                            "/etc/xdg",
                        ],
                    ),
                },
                {
                    "LD_LIBRARY_PATH": prepend_to_env(
                        "LD_LIBRARY_PATH",
                        [
                            # Qt6 arch specific libs
                            f"/snap/{qt6_sdk_snap}/current/usr/lib/"
                            "${CRAFT_ARCH_TRIPLET_BUILD_FOR}",
                            # Qt6 libs
                            f"/snap/{qt6_sdk_snap}/current/usr/lib",
                            # kf6 arch specific libs
                            f"/snap/{kf6_sdk_snap}/current/usr/lib/"
                            "${CRAFT_ARCH_TRIPLET_BUILD_FOR}",
                            # blas
                            f"/snap/{kf6_sdk_snap}/current/usr/lib/"
                            "${CRAFT_ARCH_TRIPLET_BUILD_FOR}/blas",
                            # lapack
                            f"/snap/{kf6_sdk_snap}/current/usr/lib/"
                            "${CRAFT_ARCH_TRIPLET_BUILD_FOR}/lapack",
                            # kf6 libs
                            f"/snap/{kf6_sdk_snap}/current/usr/lib",
                            # Staged libs
                            "$CRAFT_STAGE/usr/lib/${CRAFT_ARCH_TRIPLET_BUILD_FOR}",
                            "$CRAFT_STAGE/usr/lib",
                            "$CRAFT_STAGE/lib/",
                        ],
                    ),
                },
                {
                    "CMAKE_PREFIX_PATH": prepend_to_env(
                        "CMAKE_PREFIX_PATH",
                        [
                            "$CRAFT_STAGE",
                            f"/snap/{qt6_sdk_snap}/current",
                            f"/snap/{kf6_sdk_snap}/current",
                            "/usr",
                        ],
                        separator=";",
                    ),
                },
                {
                    "CMAKE_FIND_ROOT_PATH": prepend_to_env(
                        "CMAKE_FIND_ROOT_PATH",
                        [
                            "$CRAFT_STAGE",
                            f"/snap/{qt6_sdk_snap}/current",
                            f"/snap/{kf6_sdk_snap}/current",
                            "/usr",
                        ],
                        separator=";",
                    ),
                },
            ],
        }

    @overrides
    def get_parts_snippet(self) -> Dict[str, Any]:
        # We can change this to the lightweight command-chain when
        # the content snap includes the desktop-launch from
        # https://github.com/snapcore/snapcraft-desktop-integration
        source = get_extensions_data_dir() / "desktop" / "kde-neon-6"

        if self.kde_snaps.kf6_builtin:
            return {
                "kde-neon-6/sdk": {
                    "source": str(source),
                    "plugin": "make",
                    "make-parameters": [f"PLATFORM_PLUG={self.kde_snaps.content_kf6}"],
                    "build-snaps": [
                        self.kde_snaps.qt6_sdk_snap,
                        self.kde_snaps.kf6_sdk_snap,
                    ],
                    "build-packages": [
                        "gettext",
                        "doxygen",
                        "graphviz",
                        "libxml2-utils",
                        "docbook-xml",
                        "docbook-xsl",
                        "libglx-dev",
                        "libgl-dev",
                        "libglvnd-dev",
                    ],
                },
            }

        return {
            "kde-neon-6/sdk": {
                "source": str(source),
                "plugin": "make",
                "make-parameters": [f"PLATFORM_PLUG={self.kde_snaps.content_kf6}"],
            },
        }
