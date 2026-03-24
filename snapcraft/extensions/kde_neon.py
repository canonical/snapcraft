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

from typing_extensions import override

from .extension import get_extensions_data_dir, prepend_to_env
from .gpu_extension import GPUExtension

_QT5_SDK_SNAP = {"core22": "kde-qt5-core22-sdk", "core24": "kde-qt5-core24-sdk"}
_KF5_SDK_SNAP = {"core22": "kf5-core22-sdk", "core24": "kf5-core24-sdk"}


@dataclasses.dataclass
class KDESnaps:
    """A structure of KDE related snaps.

    :cvar qt5_sdk_snap: The name of the qt5 SDK snap to use.
    :cvar kf5_sdk_snap: The name of the kf5 SDK snap to use.
    :cvar content_qt5: The name of the qt5 content snap to use.
    :cvar content_kf5: The name of the kf5 content snap to use.
    :cvar qt5_builtin: True if the SDK is built into the qt5 content snap.
    :cvar kf5_builtin: True if the SDK is built into the kf5 content snap.
    """

    qt5_sdk_snap: str
    kf5_sdk_snap: str
    content_qt5: str
    content_kf5: str
    qt5_builtin: bool = True
    kf5_builtin: bool = True


class KDENeon(GPUExtension):
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
    @override
    def get_supported_bases() -> tuple[str, ...]:
        return ("core22", "core24")

    @staticmethod
    @override
    def get_supported_confinement() -> tuple[str, ...]:
        return "strict", "devmode"

    @staticmethod
    @override
    def is_experimental(base: str | None) -> bool:
        return False

    @override
    def get_app_snippet(self, *, app_name: str) -> dict[str, Any]:
        if self.yaml_data["base"] == "core24":
            snippet = super().get_app_snippet(app_name=app_name)
        else:
            snippet = {}
        snippet["command-chain"] = [
            *snippet.get("command-chain", []),
            "snap/command-chain/desktop-launch",
        ]
        snippet["plugs"] = [
            "desktop",
            "desktop-legacy",
            "opengl",
            "wayland",
            "x11",
            "audio-playback",
            "unity7",
            "network",
            "network-bind",
        ]
        return snippet

    @functools.cached_property
    def kde_snaps(self) -> KDESnaps:
        """Return the KDE related snaps to use to construct the environment."""
        base = self.yaml_data["base"]
        qt5_sdk_snap = _QT5_SDK_SNAP[base]
        kf5_sdk_snap = _KF5_SDK_SNAP[base]

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
        )

    @override
    def get_root_snippet(self) -> dict[str, Any]:
        platform_kf5_snap = self.kde_snaps.content_kf5
        content_kf5_snap = self.kde_snaps.content_kf5 + "-all"
        base = self.yaml_data["base"]

        snippet: dict[str, Any] = {}
        if base == "core24":
            snippet = super().get_root_snippet()
        else:
            snippet = {
                "layout": {
                    "/usr/share/libdrm": {"bind": "$SNAP/kf5-core22/usr/share/libdrm"}
                }
            }

        snippet["assumes"] = [
            *snippet.get("assumes", []),
            "snapd2.58.3",  # for cups support
        ]
        snippet["compression"] = "lzo"
        snippet["plugs"] = {
            **snippet.get("plugs", {}),
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
        }
        snippet["environment"] = {
            **snippet.get("environment", {}),
            "SNAP_DESKTOP_RUNTIME": "$SNAP/kf5",
            "GTK_USE_PORTAL": "1",
            "PLATFORM_PLUG": platform_kf5_snap,
        }
        snippet["hooks"] = {
            **snippet.get("hooks", {}),
            "configure": {
                "plugs": ["desktop"],
                "command-chain": ["snap/command-chain/hooks-configure-desktop"],
            },
        }
        snippet["layout"] = {
            **{
                k: v
                for k, v in snippet.get("layout", {}).items()
                if not k.startswith("/usr/share/X11")
            },
            "/usr/share/X11": {"symlink": "$SNAP/kf5/usr/share/X11"},
            "/usr/share/qt5": {"symlink": "$SNAP/kf5/usr/share/qt5"},
        }
        return snippet

    @override
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

    @override
    def get_parts_snippet(self) -> dict[str, Any]:
        """Get the parts snippet for the KDE extension.

        If the KDE Neon SDK is not built into the content snap, the add the
        sdk snap as a build-snap.
        """
        # We can change this to the lightweight command-chain when
        # the content snap includes the desktop-launch from
        # https://github.com/canonical/snapcraft-desktop-integration

        source = get_extensions_data_dir() / "desktop" / "command-chain-kde"

        base = self.yaml_data["base"]
        if base != "core22":
            parts = {f"kde-neon/{k}": v for k, v in super().get_parts_snippet().items()}
        else:
            parts = {}

        parts.update(
            {
                "kde-neon/sdk": {
                    "source": str(source),
                    "plugin": "make",
                    "make-parameters": [
                        f"PLATFORM_PLUG=kf5-{base}",
                    ],
                },
            }
        )

        if self.kde_snaps.kf5_builtin:
            parts["kde-neon/sdk"].update(
                {
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
                }
            )

        return parts
