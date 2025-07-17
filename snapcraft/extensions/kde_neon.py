# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022,2024 Canonical Ltd.
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
from typing import Any

from overrides import overrides

from .extension import Extension, get_extensions_data_dir, prepend_to_env

_QT5_SDK_SNAP = {"core22": "kde-qt5-core22-sdk", "core24": "kde-qt5-core24-sdk"}
_KF5_SDK_SNAP = {"core22": "kf5-core22-sdk", "core24": "kf5-core24-sdk"}


@dataclasses.dataclass
class KDESnaps:
    """A structure of KDE related snaps.

    :cvar qt5_sdk_snap: The name of the qt5 SDK snap to use.
    :cvar kf5_sdk_snap: The name of the kf5 SDK snap to use.
    :cvar content_qt5: The name of the qt5 content snap to use.
    :cvar content_kf5: The name of the kf5 content snap to use.
    :cvar gpu_plugs: The gpu plugs to use with gpu-2404.
    :cvar gpu_layouts: The gpu layouts to use with gpu-2404.
    :cvar qt5_builtin: True if the SDK is built into the qt5 content snap.
    :cvar kf5_builtin: True if the SDK is built into the kf5 content snap.
    """

    qt5_sdk_snap: str
    kf5_sdk_snap: str
    content_qt5: str
    content_kf5: str
    gpu_plugs: dict[str, Any]
    gpu_layouts: dict[str, Any]
    qt5_builtin: bool = True
    kf5_builtin: bool = True


class KDENeon(Extension):
    r"""The KDE Neon extension.

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
    def kde_snaps(self) -> KDESnaps:
        """Return the KDE related snaps to use to construct the environment."""
        base = self.yaml_data["base"]
        qt5_sdk_snap = _QT5_SDK_SNAP[base]
        kf5_sdk_snap = _KF5_SDK_SNAP[base]

        match base:
            case "core22":
                gpu_plugs = {}
                gpu_layouts = {
                    "/usr/share/libdrm": {"bind": "$SNAP/kf5-core22/usr/share/libdrm"},
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

        matcher = re.compile(r"kde-qt5-" + base + r"-sdk.*")
        qt5_sdk_snap_candidates = [s for s in build_snaps if matcher.match(s)]
        if qt5_sdk_snap_candidates:
            qt5_sdk_snap = qt5_sdk_snap_candidates[0].split("/")[0]
            qt5_builtin = False
        else:
            qt5_builtin = True

        matcher = re.compile(r"kf5-" + base + r"-sdk.*")
        kf5_sdk_snap_candidates = [s for s in build_snaps if matcher.match(s)]
        if kf5_sdk_snap_candidates:
            kf5_sdk_snap = kf5_sdk_snap_candidates[0].split("/")[0]
            kf5_builtin = False
        else:
            kf5_builtin = True
        # The same except the trailing -sdk
        content_qt5_snap = qt5_sdk_snap[:-4]
        content_kf5_snap = kf5_sdk_snap[:-4]

        return KDESnaps(
            qt5_sdk_snap=qt5_sdk_snap,
            content_qt5=content_qt5_snap,
            qt5_builtin=qt5_builtin,
            kf5_sdk_snap=kf5_sdk_snap,
            content_kf5=content_kf5_snap,
            kf5_builtin=kf5_builtin,
            gpu_layouts=gpu_layouts,
            gpu_plugs=gpu_plugs,
        )

    @overrides
    def get_root_snippet(self) -> dict[str, Any]:
        platform_kf5_snap = self.kde_snaps.content_kf5
        content_kf5_snap = self.kde_snaps.content_kf5 + "-all"
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
                platform_kf5_snap: {
                    "content": content_kf5_snap,
                    "interface": "content",
                    "default-provider": platform_kf5_snap,
                    "target": "$SNAP/kf5",
                },
                **gpu_plugs,
            },
            "environment": {
                "SNAP_DESKTOP_RUNTIME": "$SNAP/kf5",
                "GTK_USE_PORTAL": "1",
                "PLATFORM_PLUG": platform_kf5_snap,
            },
            "hooks": {
                "configure": {
                    "plugs": ["desktop"],
                    "command-chain": ["snap/command-chain/hooks-configure-desktop"],
                }
            },
            "layout": {
                "/usr/share/X11": {"symlink": "$SNAP/kf5/usr/share/X11"},
                "/usr/share/qt5": {"symlink": "$SNAP/kf5/usr/share/qt5"},
                **gpu_layouts,
            },
        }

    @overrides
    def get_part_snippet(self, *, plugin_name: str) -> dict[str, Any]:
        qt5_sdk_snap = self.kde_snaps.qt5_sdk_snap
        kf5_sdk_snap = self.kde_snaps.kf5_sdk_snap

        if self.yaml_data["base"] == "core24":
            return {
                "build-environment": [
                    {
                        "PATH": prepend_to_env(
                            "PATH",
                            [
                                f"/snap/{qt5_sdk_snap}/current/usr/bin",
                                f"/snap/{kf5_sdk_snap}/current/usr/bin",
                            ],
                        ),
                    },
                    {
                        "XDG_DATA_DIRS": prepend_to_env(
                            "XDG_DATA_DIRS",
                            [
                                "$CRAFT_STAGE/usr/share",
                                f"/snap/{qt5_sdk_snap}/current/usr/share",
                                f"/snap/{kf5_sdk_snap}/current/usr/share",
                            ],
                        ),
                    },
                    {
                        "XDG_CONFIG_HOME": prepend_to_env(
                            "XDG_CONFIG_HOME",
                            [
                                "$CRAFT_STAGE/etc/xdg",
                                f"/snap/{qt5_sdk_snap}/current/etc/xdg",
                                f"/snap/{kf5_sdk_snap}/current/etc/xdg",
                            ],
                        ),
                    },
                    {
                        "LD_LIBRARY_PATH": prepend_to_env(
                            "LD_LIBRARY_PATH",
                            [
                                # Qt5 arch specific libs
                                f"/snap/{qt5_sdk_snap}/current/usr/lib/"
                                "${CRAFT_ARCH_TRIPLET_BUILD_FOR}",
                                # Qt5 libs
                                f"/snap/{qt5_sdk_snap}/current/usr/lib",
                                # kf5 arch specific libs
                                f"/snap/{kf5_sdk_snap}/current/usr/lib/"
                                "${CRAFT_ARCH_TRIPLET_BUILD_FOR}",
                                # Mesa libs
                                "/snap/mesa-2404/current/usr/lib/"
                                "${CRAFT_ARCH_TRIPLET_BUILD_FOR}",
                                # blas
                                f"/snap/{kf5_sdk_snap}/current/usr/lib/"
                                "${CRAFT_ARCH_TRIPLET_BUILD_FOR}/blas",
                                # lapack
                                f"/snap/{kf5_sdk_snap}/current/usr/lib/"
                                "${CRAFT_ARCH_TRIPLET_BUILD_FOR}/lapack",
                                # kf5 libs
                                f"/snap/{kf5_sdk_snap}/current/usr/lib",
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
                                f"/snap/{qt5_sdk_snap}/current/usr",
                                f"/snap/{kf5_sdk_snap}/current/usr",
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
                            f"/snap/{qt5_sdk_snap}/current/usr/bin",
                            f"/snap/{kf5_sdk_snap}/current/usr/bin",
                        ],
                    ),
                },
                {
                    "XDG_DATA_DIRS": prepend_to_env(
                        "XDG_DATA_DIRS",
                        [
                            "$CRAFT_STAGE/usr/share",
                            f"/snap/{qt5_sdk_snap}/current/usr/share",
                            f"/snap/{kf5_sdk_snap}/current/usr/share",
                        ],
                    ),
                },
                {
                    "XDG_CONFIG_HOME": prepend_to_env(
                        "XDG_CONFIG_HOME",
                        [
                            "$CRAFT_STAGE/etc/xdg",
                            f"/snap/{qt5_sdk_snap}/current/etc/xdg",
                            f"/snap/{kf5_sdk_snap}/current/etc/xdg",
                        ],
                    ),
                },
                {
                    "LD_LIBRARY_PATH": prepend_to_env(
                        "LD_LIBRARY_PATH",
                        [
                            # Qt5 arch specific libs
                            f"/snap/{qt5_sdk_snap}/current/usr/lib/"
                            "${CRAFT_ARCH_TRIPLET_BUILD_FOR}",
                            # Qt5 libs
                            f"/snap/{qt5_sdk_snap}/current/usr/lib",
                            # kf5 arch specific libs
                            f"/snap/{kf5_sdk_snap}/current/usr/lib/"
                            "${CRAFT_ARCH_TRIPLET_BUILD_FOR}",
                            # blas
                            f"/snap/{kf5_sdk_snap}/current/usr/lib/"
                            "${CRAFT_ARCH_TRIPLET_BUILD_FOR}/blas",
                            # lapack
                            f"/snap/{kf5_sdk_snap}/current/usr/lib/"
                            "${CRAFT_ARCH_TRIPLET_BUILD_FOR}/lapack",
                            # kf5 libs
                            f"/snap/{kf5_sdk_snap}/current/usr/lib",
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
                            f"/snap/{qt5_sdk_snap}/current/usr",
                            f"/snap/{kf5_sdk_snap}/current/usr",
                            "/usr",
                        ],
                        separator=":",
                    ),
                },
            ],
        }

    @overrides
    def get_parts_snippet(self) -> dict[str, Any]:
        """Get the parts snippet for the KDE extension.

        If the KDE Neon SDK is not built into the content snap, the add the
        sdk snap as a build-snap.
        """
        # We can change this to the lightweight command-chain when
        # the content snap includes the desktop-launch from
        # https://github.com/canonical/snapcraft-desktop-integration

        source = get_extensions_data_dir() / "desktop" / "command-chain-kde"

        gpu_opts = {}
        if self.yaml_data["base"] == "core24":
            gpu_opts["make-parameters"] = [
                "GPU_WRAPPER=gpu-2404-wrapper",
                "PLATFORM_PLUG=kf5-core24",
            ]
        else:
            gpu_opts["make-parameters"] = [
                "PLATFORM_PLUG=kf5-core22",
            ]

        if self.kde_snaps.kf5_builtin:
            return {
                "kde-neon/sdk": {
                    "source": str(source),
                    "plugin": "make",
                    "build-snaps": [
                        self.kde_snaps.qt5_sdk_snap,
                        self.kde_snaps.kf5_sdk_snap,
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
            "kde-neon/sdk": {
                "source": str(source),
                "plugin": "make",
                **gpu_opts,
            },
        }
