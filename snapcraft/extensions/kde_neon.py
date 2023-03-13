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


@dataclasses.dataclass
class ExtensionInfo:
    """Content/SDK build information."""

    cmake_args: str


@dataclasses.dataclass
class KDESnaps:
    """A structure of KDE related snaps."""

    sdk: str
    content: str
    builtin: bool = True


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
        }

    @functools.cached_property
    def kde_snaps(self) -> KDESnaps:
        """Return the KDE related snaps to use to construct the environment."""
        base = self.yaml_data["base"]
        sdk_snap = _SDK_SNAP[base]

        build_snaps: List[str] = []
        for part in self.yaml_data["parts"].values():
            build_snaps.extend(part.get("build-snaps", []))

        matcher = re.compile(r"kde-frameworks-\d+-\d+-qt-\d+.*-" + base + r"-sd.*")
        sdk_snap_candidates = [s for s in build_snaps if matcher.match(s)]
        if sdk_snap_candidates:
            sdk_snap = sdk_snap_candidates[0].split("/")[0]
            builtin = False
        else:
            builtin = True
        # The same except the trailing -sd
        content = sdk_snap[:-3]

        return KDESnaps(sdk=sdk_snap, content=content, builtin=builtin)

    @functools.cached_property
    def ext_info(self) -> ExtensionInfo:
        """Return the extension info cmake_args, provider, content, build_snaps."""
        cmake_args = "-DCMAKE_FIND_ROOT_PATH=/snap/" + self.kde_snaps.sdk + "/current"

        return ExtensionInfo(cmake_args=cmake_args)

    @overrides
    def get_root_snippet(self) -> Dict[str, Any]:
        platform_snap = self.kde_snaps.content
        content_snap = self.kde_snaps.content + "-all"

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
                platform_snap: {
                    "content": content_snap,
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
            "layout": {"/usr/share/X11": {"symlink": "$SNAP/kf5/usr/share/X11"}},
        }

    @overrides
    def get_part_snippet(self) -> Dict[str, Any]:
        sdk_snap = self.kde_snaps.sdk
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
                    "ACLOCAL_PATH": prepend_to_env(
                        "ACLOCAL_PATH",
                        [
                            f"/snap/{sdk_snap}/current/usr/share/aclocal",
                        ],
                    ),
                },
                {
                    "SNAPCRAFT_CMAKE_ARGS": prepend_to_env(
                        "SNAPCRAFT_CMAKE_ARGS",
                        [
                            cmake_args,
                        ],
                    ),
                },
            ],
        }

    @overrides
    def get_parts_snippet(self) -> Dict[str, Any]:
        # We can change this to the lightweight command-chain when
        # the content snap includes the desktop-launch from
        # https://github.com/snapcore/snapcraft-desktop-integration
        source = get_extensions_data_dir() / "desktop" / "kde-neon"

        if self.kde_snaps.builtin:
            return {
                "kde-neon/sdk": {
                    "source": str(source),
                    "plugin": "make",
                    "make-parameters": [f"PLATFORM_PLUG={self.kde_snaps.content}"],
                    "build-snaps": [self.kde_snaps.sdk],
                },
            }

        return {
            "kde-neon/sdk": {
                "source": str(source),
                "plugin": "make",
                "make-parameters": [f"PLATFORM_PLUG={self.kde_snaps.content}"],
            },
        }
