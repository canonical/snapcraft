# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022,2024 Canonical Ltd.
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

"""Generic GNOME extension to support core22 and onwards."""

import dataclasses
import functools
import re
from typing import Any

from overrides import overrides

from .extension import Extension, get_extensions_data_dir, prepend_to_env

_SDK_SNAP = {"core22": "gnome-42-2204-sdk", "core24": "gnome-46-2404-sdk"}
_PLATFORM_TRANSLATION = {"core22": "2204", "core24": "2404"}


@dataclasses.dataclass
class GNOMESnaps:
    """A structure of GNOME related snaps.

    :cvar sdk: The name of the SDK snap to use.
    :cvar content: The name of the content snap to use.
    :cvar builtin: True if the SDK is built into the content snap.
    """

    sdk: str
    content: str
    builtin: bool = True


class GNOME(Extension):
    """An extension that eases the creation of snaps that integrate with GNOME.

    When used with core22 GNOME 42 will be used.

    At build time it ensures the right build dependencies are setup and for
    the runtime it ensures the application is run in an environment catered
    for GNOME applications.

    It configures each application with the following plugs:

    - GTK3 Themes.
    - Common Icon Themes.
    - Common Sound Themes.
    - The GNOME runtime libraries and utilities corresponding to 3.38.

    For easier desktop integration, it also configures each application
    entry with these additional plugs:

    - desktop (https://snapcraft.io/docs/desktop-interface)
    - desktop-legacy (https://snapcraft.io/docs/desktop-legacy-interface)
    - gsettings (https://snapcraft.io/docs/gsettings-interface)
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
                "gsettings",
                "opengl",
                "wayland",
                "x11",
            ],
        }

    @functools.cached_property
    def gnome_snaps(self) -> GNOMESnaps:
        """Return the GNOME related snaps to use to construct the environment."""
        base = self.yaml_data["base"]
        sdk_snap = _SDK_SNAP[base]

        build_snaps: list[str] = []
        for part in self.yaml_data["parts"].values():
            build_snaps.extend(part.get("build-snaps", []))

        # use the sdk snap if it is defined in any part's build-snaps
        # otherwise, assume it is built into the content snap
        matcher = re.compile(r"gnome-\d+-" + _PLATFORM_TRANSLATION[base] + r"-sdk.*")
        sdk_snap_candidates = [s for s in build_snaps if matcher.match(s)]
        if sdk_snap_candidates:
            sdk_snap = sdk_snap_candidates[0].split("/")[0]
            builtin = False
        else:
            builtin = True
        # The same except the trailing -sdk
        content = sdk_snap[:-4]

        return GNOMESnaps(sdk=sdk_snap, content=content, builtin=builtin)

    @overrides
    def get_root_snippet(self) -> dict[str, Any]:
        platform_snap = self.gnome_snaps.content
        base = self.yaml_data["base"]

        match base:
            case "core22":
                gpu_plugs = {}
                gpu_layouts = {
                    "/usr/share/libdrm": {
                        "bind": "$SNAP/gnome-platform/usr/share/libdrm"
                    },
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
                    "/usr/share/X11/XErrorDB": {
                        "symlink": "$SNAP/gpu-2404/X11/XErrorDB"
                    },
                }
            case _:
                raise AssertionError(f"Unsupported base: {base}")

        return {
            "assumes": ["snapd2.43"],  # for 'snapctl is-connected'
            "plugs": {
                "desktop": {"mount-host-font-cache": False},
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
                platform_snap: {
                    "interface": "content",
                    "target": "$SNAP/gnome-platform",
                    "default-provider": platform_snap,
                },
                **gpu_plugs,
            },
            "environment": {
                "SNAP_DESKTOP_RUNTIME": "$SNAP/gnome-platform",
                "GTK_USE_PORTAL": "1",
            },
            "hooks": {
                "configure": {
                    "plugs": ["desktop"],
                    "command-chain": ["snap/command-chain/hooks-configure-fonts"],
                }
            },
            "layout": {
                "/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.0": {
                    "bind": (
                        "$SNAP/gnome-platform/usr/lib/"
                        "$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.0"
                    )
                },
                "/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.1": {
                    "bind": (
                        "$SNAP/gnome-platform/usr/lib/"
                        "$CRAFT_ARCH_TRIPLET_BUILD_FOR/webkit2gtk-4.1"
                    )
                },
                "/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR/libproxy": {
                    "bind": (
                        "$SNAP/gnome-platform/usr/lib/"
                        "$CRAFT_ARCH_TRIPLET_BUILD_FOR/libproxy"
                    )
                },
                "/usr/share/xml/iso-codes": {
                    "bind": "$SNAP/gnome-platform/usr/share/xml/iso-codes"
                },
                **gpu_layouts,
            },
        }

    @overrides
    def get_part_snippet(self, *, plugin_name: str) -> dict[str, Any]:
        sdk_snap = self.gnome_snaps.sdk

        return {
            "build-environment": [
                {
                    "SNAPCRAFT_GNOME_SDK": f"/snap/{sdk_snap}/current/",
                },
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
                    "LD_LIBRARY_PATH": prepend_to_env(
                        "LD_LIBRARY_PATH",
                        [
                            f"/snap/{sdk_snap}/current/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR",
                            f"/snap/{sdk_snap}/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR",
                            f"/snap/{sdk_snap}/current/usr/lib",
                            f"/snap/{sdk_snap}/current/usr/lib/vala-current",
                            (
                                f"/snap/{sdk_snap}/current/usr/lib/"
                                "$CRAFT_ARCH_TRIPLET_BUILD_FOR/pulseaudio"
                            ),
                        ],
                    ),
                },
                {
                    "PKG_CONFIG_PATH": prepend_to_env(
                        "PKG_CONFIG_PATH",
                        [
                            (
                                f"/snap/{sdk_snap}/current/usr/lib/"
                                "$CRAFT_ARCH_TRIPLET_BUILD_FOR/pkgconfig"
                            ),
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
                    "GDK_PIXBUF_MODULE_FILE": (
                        f"/snap/{sdk_snap}/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR"
                        "/gdk-pixbuf-current/loaders.cache"
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
                            f"/snap/{sdk_snap}/current/usr/lib/$CRAFT_ARCH_TRIPLET_BUILD_FOR"
                            "/gobject-introspection",
                        ],
                    ),
                },
                {
                    "GI_TYPELIB_PATH": prepend_to_env(
                        "GI_TYPELIB_PATH",
                        [
                            f"/snap/{sdk_snap}/current/usr/lib/girepository-1.0",
                            (
                                f"/snap/{sdk_snap}/current/usr/lib/"
                                "$CRAFT_ARCH_TRIPLET_BUILD_FOR/girepository-1.0"
                            ),
                        ],
                    )
                },
                {
                    "CMAKE_PREFIX_PATH": prepend_to_env(
                        "CMAKE_PREFIX_PATH",
                        [
                            "$CRAFT_STAGE/usr",
                            f"/snap/{sdk_snap}/current/usr",
                        ],
                        separator=":",
                    ),
                },
            ],
        }

    @overrides
    def get_parts_snippet(self) -> dict[str, Any]:
        """Get the parts snippet for the GNOME extension.

        If the GNOME SDK is not built into the content snap, the add the
        sdk snap as a build-snap.
        """
        source = get_extensions_data_dir() / "desktop" / "command-chain"

        gpu_opts = {}
        if self.yaml_data["base"] == "core24":
            gpu_opts["make-parameters"] = ["GPU_WRAPPER=gpu-2404-wrapper"]

        if self.gnome_snaps.builtin:
            base = self.yaml_data["base"]
            sdk_snap = _SDK_SNAP[base]
            return {
                "gnome/sdk": {
                    "source": str(source),
                    "plugin": "make",
                    "build-snaps": [sdk_snap],
                    **gpu_opts,
                },
            }

        return {
            "gnome/sdk": {
                "source": str(source),
                "plugin": "make",
                **gpu_opts,
            },
        }
