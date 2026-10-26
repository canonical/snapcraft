# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
#           2023-2026 Scarlett Moore <sgmoore@kde.org>
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

from craft_application.errors import CraftValidationError
from typing_extensions import override

from .extension import get_extensions_data_dir, prepend_to_env
from .gpu_extension import GPUExtension

_QT6_SDK_SNAP = {
    "core22": "kde-qt6-core22-sdk",
    "core24": "kde-qt6-core24-sdk",
    "core26": "kde-qt6-core26-sdk",
}
_KF6_SDK_SNAP = {
    "core22": "kf6-core22-sdk",
    "core24": "kf6-core24-sdk",
    "core26": "kf6-core26-sdk",
}


@dataclasses.dataclass
class KDESnaps6:
    """A structure of KDE related snaps.

    :cvar qt6_sdk_snap: The name of the qt6 SDK snap to use.
    :cvar kf6_sdk_snap: The name of the kf6 SDK snap to use.
    :cvar content_qt6: The name of the qt6 content snap to use.
    :cvar content_kf6: The name of the kf6 content snap to use.
    :cvar qt6_builtin: True if the SDK is built into the qt6 content snap.
    :cvar kf6_builtin: True if the SDK is built into the kf6 content snap.
    """

    qt6_sdk_snap: str
    kf6_sdk_snap: str
    content_qt6: str
    content_kf6: str
    qt6_builtin: bool = True
    kf6_builtin: bool = True


class KDENeon6(GPUExtension):
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
    - desktop (https://snapcraft.io/docs/reference/interfaces/desktop-interface)
    - desktop-legacy (https://snapcraft.io/docs/reference/interfaces/desktop-legacy-interface)
    - opengl (https://snapcraft.io/docs/reference/interfaces/opengl-interface)
    - wayland (https://snapcraft.io/docs/reference/interfaces/wayland-interface)
    - x11 (https://snapcraft.io/docs/reference/interfaces/x11-interface)
    - audio-playback (https://snapcraft.io/docs/reference/interfaces/audio-playback-interface)
    - unity7 (https://snapcraft.io/docs/reference/interfaces/unity7-interface)
    - network (https://snapcraft.io/docs/reference/interfaces/network-interface)
    - network-bind (https://snapcraft.io/docs/reference/interfaces/network-bind-interface)
    """

    @staticmethod
    @override
    def get_supported_bases() -> tuple[str, ...]:
        return ("core22", "core24", "core26")

    @staticmethod
    @override
    def get_supported_confinement() -> tuple[str, ...]:
        return "strict", "devmode"

    @staticmethod
    @override
    def is_experimental(base: str | None) -> bool:
        # Core 26 support is preliminary while the KDE runtime stack settles.
        return base == "core26"

    @override
    def get_app_snippet(self, *, app_name: str) -> dict[str, Any]:
        if self.yaml_data["base"] in ("core24", "core26"):
            snippet = super().get_app_snippet(app_name=app_name)
        else:
            snippet = {}
        snippet["command-chain"] = [
            *snippet.get("command-chain", []),
            "snap/command-chain/desktop-launch",
        ]
        snippet["plugs"] = [
            *snippet.get("plugs", []),
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
    def kde_snaps(self) -> KDESnaps6:
        """Return the KDE related snaps to use to construct the environment."""
        base = self.yaml_data["base"]
        qt6_sdk_snap = _QT6_SDK_SNAP[base]
        kf6_sdk_snap = _KF6_SDK_SNAP[base]

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
        )

    @functools.cached_property
    def pyside_sdk_snap(self) -> str | None:
        """Return an explicitly configured PySide6 SDK snap, if any.

        PySide6 is optional and is not part of the KDE Neon extension's
        default runtime. Only add its build paths for core26 projects that
        explicitly list the matching SDK in a part's build-snaps.
        """
        if self.yaml_data.get("base") != "core26":
            return None

        matcher = re.compile(r"kde-pyside6-core26-sdk.*")
        for part in self.yaml_data.get("parts", {}).values():
            for snap in part.get("build-snaps", []):
                if matcher.match(snap):
                    return snap.split("/")[0]
        return None

    @functools.cached_property
    def kf6_sdk_channel(self) -> str | None:
        """Return the configured KF6 SDK channel, if one was explicitly set."""
        if self.yaml_data.get("base") != "core26":
            return None

        matcher = re.compile(r"kf6-core26-sdk(?:/(.+))?")
        for part in self.yaml_data.get("parts", {}).values():
            for snap in part.get("build-snaps", []):
                match = matcher.fullmatch(snap)
                if match:
                    return match.group(1)
        return None

    @override
    def get_root_snippet(self) -> dict[str, Any]:
        platform_kf6_snap = self.kde_snaps.content_kf6
        content_kf6_snap = self.kde_snaps.content_kf6 + "-all"
        if not (base := self.yaml_data.get("base")):
            raise CraftValidationError("The 'base' key is required.")
        lxqt_support_snap = f"lxqt-support-{base}"

        snippet: dict[str, Any] = {}
        if base in ("core24", "core26"):
            snippet = super().get_root_snippet()
        else:
            snippet = {
                "layout": {
                    "/usr/share/libdrm": {"bind": "$SNAP/kf6-core22/usr/share/libdrm"}
                }
            }

        snippet["assumes"] = ["snapd2.58.3"]  # for cups support
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
            platform_kf6_snap: {
                "content": content_kf6_snap,
                "interface": "content",
                "default-provider": platform_kf6_snap,
                "target": "$SNAP/kf6",
            },
            lxqt_support_snap: {
                "content": lxqt_support_snap,
                "interface": "content",
                "target": "$SNAP/lxqt-support",
                "default-provider": lxqt_support_snap,
            },
        }
        snippet["environment"] = {
            "SNAP_DESKTOP_RUNTIME": "$SNAP/kf6",
            "GTK_USE_PORTAL": "1",
            "PLATFORM_PLUG": platform_kf6_snap,
        }
        snippet["hooks"] = {
            "configure": {
                "plugs": ["desktop"],
                "command-chain": ["snap/command-chain/hooks-configure-desktop"],
            }
        }
        snippet["layout"] = {
            **{
                k: v
                for k, v in snippet.get("layout", {}).items()
                if not (
                    k.startswith("/usr/share/X11")
                    or (
                        base in ("core24", "core26")
                        and k in ("/usr/share/libdrm", "/usr/share/drirc.d")
                    )
                )
            },
            "/usr/share/X11": {"symlink": "$SNAP/kf6/usr/share/X11"},
            "/usr/share/qt6": {"symlink": "$SNAP/kf6/usr/share/qt6"},
            "/usr/share/Kvantum": {"symlink": "$SNAP/lxqt-support/usr/share/Kvantum"},
        }
        return snippet

    @override
    def get_part_snippet(self, *, plugin_name: str) -> dict[str, Any]:
        qt6_sdk_snap = self.kde_snaps.qt6_sdk_snap
        kf6_sdk_snap = self.kde_snaps.kf6_sdk_snap
        mesa_snap = {
            "core22": "mesa-2204",
            "core24": "mesa-2404",
            "core26": "mesa-2604",
        }[self.yaml_data["base"]]

        if self.yaml_data["base"] in ("core24", "core26"):
            pyside_sdk_snap = self.pyside_sdk_snap
            pyside_paths = (
                [f"/snap/{pyside_sdk_snap}/current/usr"] if pyside_sdk_snap else []
            )
            pyside_library_paths = (
                [
                    f"/snap/{pyside_sdk_snap}/current/usr/lib",
                    f"/snap/{pyside_sdk_snap}/current/usr/lib/"
                    "${CRAFT_ARCH_TRIPLET_BUILD_FOR}",
                ]
                if pyside_sdk_snap
                else []
            )
            environment = [
                {
                    "PATH": prepend_to_env(
                        "PATH",
                        [
                            "$CRAFT_STAGE/usr/bin",
                            f"/snap/{qt6_sdk_snap}/current/usr/bin",
                            f"/snap/{kf6_sdk_snap}/current/usr/bin",
                            *(
                                [f"/snap/{pyside_sdk_snap}/current/usr/bin"]
                                if pyside_sdk_snap
                                else []
                            ),
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
                            # Mesa libs supplied by the base-specific GPU snap.
                            f"/snap/{mesa_snap}/current/usr/lib/"
                            "${CRAFT_ARCH_TRIPLET_BUILD_FOR}",
                            # blas
                            f"/snap/{kf6_sdk_snap}/current/usr/lib/"
                            "${CRAFT_ARCH_TRIPLET_BUILD_FOR}/blas",
                            # lapack
                            f"/snap/{kf6_sdk_snap}/current/usr/lib/"
                            "${CRAFT_ARCH_TRIPLET_BUILD_FOR}/lapack",
                            # Keep Qt/KF6 libraries ahead of PySide's libraries so
                            # PySide uses the shared KDE Qt runtime, not a duplicate.
                            *pyside_library_paths,
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
                            *pyside_paths,
                            "/usr",
                        ],
                        separator=":",
                    ),
                },
            ]

            if pyside_sdk_snap:
                environment.append(
                    {
                        "PYTHONPATH": prepend_to_env(
                            "PYTHONPATH",
                            [
                                f"/snap/{pyside_sdk_snap}/current/usr/lib/python3/dist-packages"
                            ],
                        ),
                    }
                )

            if self.yaml_data.get("base") == "core26":
                environment.append(
                    {
                        "LDFLAGS": prepend_to_env(
                            "LDFLAGS",
                            [
                                f"-Wl,-rpath-link,/snap/{qt6_sdk_snap}/current/usr/lib/"
                                "${CRAFT_ARCH_TRIPLET_BUILD_FOR}/pulseaudio",
                                f"-Wl,-rpath-link,/snap/{kf6_sdk_snap}/current/usr/lib/"
                                "${CRAFT_ARCH_TRIPLET_BUILD_FOR}/pulseaudio",
                            ],
                            separator=" ",
                        ),
                    }
                )

            return {"build-environment": environment}

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

    @override
    def get_parts_snippet(self) -> dict[str, Any]:
        base = self.yaml_data["base"]

        if base != "core22":
            parts = {
                f"kde-neon-6/{k}": v for k, v in super().get_parts_snippet().items()
            }
        else:
            parts = {}

        sdk_part = {
            "source": str(get_extensions_data_dir() / "desktop" / "command-chain-kde"),
            "plugin": "make",
            "make-parameters": [
                f"PLATFORM_PLUG=kf6-{base}",
            ],
        }

        build_snaps = []

        if self.kde_snaps.qt6_builtin:
            build_snaps.append(self.kde_snaps.qt6_sdk_snap)

        if self.kde_snaps.kf6_builtin:
            build_snaps.append(self.kde_snaps.kf6_sdk_snap)

        if build_snaps:
            sdk_part["build-snaps"] = build_snaps
            sdk_part["build-packages"] = [
                "gettext",
                "doxygen",
                "graphviz",
                "libxml2-utils",
                "docbook-xml",
                "docbook-xsl",
            ]

        # Snapcraft's content-interface default-provider only accepts a snap
        # name, not a channel. For Core26 builds, explicitly install the KF6
        # runtime as a build snap so local builds can use the intended channel.
        # Follow the configured KF6 SDK channel by default, but let projects
        # override the runtime independently by declaring kf6-core26/<channel>
        # in one of their own parts' build-snaps.
        if base == "core26" and len(build_snaps) != 1:
            runtime_snap = self.kde_snaps.content_kf6
            runtime_matcher = re.compile(re.escape(runtime_snap) + r"(?:/.*)?")

            runtime_is_explicit = False
            for part in self.yaml_data.get("parts", {}).values():
                part_build_snaps = part.get("build-snaps", [])
                if isinstance(part_build_snaps, str):
                    part_build_snaps = [part_build_snaps]

                if any(runtime_matcher.fullmatch(snap) for snap in part_build_snaps):
                    runtime_is_explicit = True
                    break

            if not runtime_is_explicit:
                runtime_channel = self.kf6_sdk_channel
                runtime_snap_ref = (
                    f"{runtime_snap}/{runtime_channel}"
                    if runtime_channel
                    else runtime_snap
                )

                sdk_build_snaps = sdk_part.get("build-snaps", [])
                if isinstance(sdk_build_snaps, str):
                    sdk_build_snaps = [sdk_build_snaps]

                sdk_build_snaps.append(runtime_snap_ref)
                sdk_part["build-snaps"] = sdk_build_snaps

        parts["kde-neon-6/sdk"] = sdk_part
        return parts

        return parts
