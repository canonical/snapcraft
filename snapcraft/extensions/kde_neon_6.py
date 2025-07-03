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
from typing import Any

from overrides import overrides

from .extension import Extension, get_extensions_data_dir, prepend_to_env

_QT6_SDK_SNAP = {"core22": "kde-qt6-core22-sdk", "core24": "kde-qt6-core24-sdk"}
_KF6_SDK_SNAP = {"core22": "kf6-core22-sdk", "core24": "kf6-core24-sdk"}


@dataclasses.dataclass
class KDESnaps6:
    """A structure of KDE related snaps.

    :cvar qt6_sdk_snap: The name of the qt6 SDK snap to use.
    :cvar kf6_sdk_snap: The name of the kf6 SDK snap to use.
    :cvar content_qt6: The name of the qt6 content snap to use.
    :cvar content_kf6: The name of the kf6 content snap to use.
    :cvar gpu_plugs: The gpu plugs to use with gpu-2404.
    :cvar gpu_layouts: The gpu layouts to use with gpu-2404.
    :cvar qt6_builtin: True if the SDK is built into the qt6 content snap.
    :cvar kf6_builtin: True if the SDK is built into the kf6 content snap.
    """

    qt6_sdk_snap: str
    kf6_sdk_snap: str
    content_qt6: str
    content_kf6: str
    gpu_plugs: dict[str, Any]
    gpu_layouts: dict[str, Any]
    qt6_builtin: bool = True
    kf6_builtin: bool = True


class KDENeon6(Extension):
    r"""The KDE Neon extension.

    This extension makes it easy to assemble KDE based applications
    using the Neon stack.

    It configures each application with the following plugs:

    \b
    - Common GTK themes.
    - breeze GTK theme.
    - Common Qt themes.
    - Common Icon Themes.
    - Breeze Icon theme.
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
    def get_supported_bases() -> tuple[str, ...]:
        return ("core22", "core24")

    @staticmethod
    @overrides
    def get_supported_confinement() -> tuple[str, ...]:
        return "strict", "devmode"

    @staticmethod
    @overrides
    def is_experimental(base: str | None) -> bool:
        return False

    @overrides
    def get_app_snippet(self, *, app_name: str) -> dict[str, Any]:
        command_chain = ["snap/command-chain/desktop-launch"]
        if self.yaml_data["base"] == "core24":
            command_chain.insert(0, "snap/command-chain/gpu-2404-wrapper")
        return {
            "command-chain": command_chain,
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

        match base:
            case "core22":
                gpu_plugs = {}
                gpu_layouts = {
                    "/usr/share/libdrm": {"bind": "$SNAP/kf6-core22/usr/share/libdrm"},
                }
            case "core24":
                gpu_plugs = {
                    "gpu-2404": {
                        "interface": "content",
                        "target": "$SNAP/gpu-2404",
                        "default-provider": "mesa-2404",
                    },
                }
                gpu_layouts = {
                    "/usr/share/libdrm": {"bind": "$SNAP/gpu-2404/libdrm"},
                    "/usr/share/drirc.d": {"symlink": "$SNAP/gpu-2404/drirc.d"},
                }
            case _:
                raise AssertionError(f"Unsupported base: {base}")

        build_snaps: list[str] = []
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
            gpu_layouts=gpu_layouts,
            gpu_plugs=gpu_plugs,
        )

    @overrides
    def get_root_snippet(self) -> dict[str, Any]:
        platform_kf6_snap = self.kde_snaps.content_kf6
        content_kf6_snap = self.kde_snaps.content_kf6 + "-all"
        gpu_plugs = self.kde_snaps.gpu_plugs
        gpu_layouts = self.kde_snaps.gpu_layouts

        return {
            "assumes": ["snapd2.58.3"],  # for cups support
            "compression": "lzo",
            "plugs": {
                "desktop": {"mount-host-font-cache": False},
                "gtk-2-themes": {
                    "interface": "content",
                    "target": "$SNAP/data-dir/themes",
                    "default-provider": "gtk-common-themes",
                },
                "gtk-3-themes": {
                    "interface": "content",
                    "target": "$SNAP/data-dir/themes",
                    "default-provider": "gtk-common-themes",
                },
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
                **gpu_plugs,
            },
            "environment": {
                "SNAP_DESKTOP_RUNTIME": "$SNAP/kf6",
                "GTK_USE_PORTAL": "1",
                "PLATFORM_PLUG": platform_kf6_snap,
            },
            "hooks": {
                "configure": {
                    "plugs": ["desktop"],
                    "command-chain": ["snap/command-chain/hooks-configure-desktop"],
                }
            },
            "layout": {
                "/usr/share/X11": {"symlink": "$SNAP/kf6/usr/share/X11"},
                "/usr/share/qt6": {"symlink": "$SNAP/kf6/usr/share/qt6"},
                **gpu_layouts,
            },
        }

    @overrides
    def get_part_snippet(self, *, plugin_name: str) -> dict[str, Any]:
        qt6_sdk_snap = self.kde_snaps.qt6_sdk_snap
        kf6_sdk_snap = self.kde_snaps.kf6_sdk_snap

        if self.yaml_data["base"] == "core24":
            return {
                "build-environment": [
                    {
                        "PATH": prepend_to_env(
                            "PATH",
                            [
                                "$CRAFT_STAGE/usr/bin",
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
                                # Mesa libs
                                "/snap/mesa-2404/current/usr/lib/"
                                "${CRAFT_ARCH_TRIPLET_BUILD_FOR}",
                                # blas
                                f"/snap/{kf6_sdk_snap}/current/usr/lib/"
                                "${CRAFT_ARCH_TRIPLET_BUILD_FOR}/blas",
                                # lapack
                                f"/snap/{kf6_sdk_snap}/current/usr/lib/"
                                "${CRAFT_ARCH_TRIPLET_BUILD_FOR}/lapack",
                                # libproxy
                                f"/snap/{qt6_sdk_snap}/current/usr/lib/"
                                "${CRAFT_ARCH_TRIPLET_BUILD_FOR}/libproxy",
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
                                "$CRAFT_STAGE/usr",
                                f"/snap/{qt6_sdk_snap}/current/usr",
                                f"/snap/{kf6_sdk_snap}/current/usr",
                                "/usr",
                            ],
                            separator=":",
                        ),
                    },
                ],
            }
        return {
            "build-environment": [
                {
                    "PATH": prepend_to_env(
                        "PATH",
                        [
                            "$CRAFT_STAGE/usr/bin",
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
                            # libproxy
                            f"/snap/{qt6_sdk_snap}/current/usr/lib/"
                            "${CRAFT_ARCH_TRIPLET_BUILD_FOR}/libproxy",
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
                            "$CRAFT_STAGE/usr",
                            f"/snap/{qt6_sdk_snap}/current/usr",
                            f"/snap/{kf6_sdk_snap}/current/usr",
                            "/usr",
                        ],
                        separator=":",
                    ),
                },
            ],
        }

    @overrides
    def get_parts_snippet(self) -> dict[str, Any]:
        # We can change this to the lightweight command-chain when
        # the content snap includes the desktop-launch from
        # https://github.com/canonical/snapcraft-desktop-integration

        source = get_extensions_data_dir() / "desktop" / "command-chain-kde"

        gpu_opts = {}
        if self.yaml_data["base"] == "core24":
            gpu_opts["make-parameters"] = [
                "GPU_WRAPPER=gpu-2404-wrapper",
                "PLATFORM_PLUG=kf6-core24",
            ]
        else:
            gpu_opts["make-parameters"] = [
                "PLATFORM_PLUG=kf6-core22",
            ]

        if self.kde_snaps.kf6_builtin:
            return {
                "kde-neon-6/sdk": {
                    "source": str(source),
                    "plugin": "make",
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
                    **gpu_opts,
                },
            }

        return {
            "kde-neon-6/sdk": {
                "source": str(source),
                "plugin": "make",
                **gpu_opts,
            },
        }
