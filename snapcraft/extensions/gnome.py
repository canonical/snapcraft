# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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


from typing import Any, Dict, Optional, Tuple

from overrides import overrides

from .extension import Extension, get_extensions_data_dir, prepend_to_env

_PLATFORM_SNAP = dict(core22="gnome-42-2204")
_SDK_SNAP = dict(core22="gnome-42-2204-sdk")


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
                "gsettings",
                "opengl",
                "wayland",
                "x11",
            ],
        }

    @overrides
    def get_root_snippet(self) -> Dict[str, Any]:
        base = self.yaml_data["base"]
        platform_snap = _PLATFORM_SNAP[base]

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
                "/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0": {
                    "bind": "$SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0"
                },
                "/usr/share/xml/iso-codes": {
                    "bind": "$SNAP/gnome-platform/usr/share/xml/iso-codes"
                },
            },
        }

    @overrides
    def get_part_snippet(self) -> Dict[str, Any]:
        base = self.yaml_data["base"]
        sdk_snap = _SDK_SNAP[base]

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
                    "GDK_PIXBUF_MODULE_FILE": (
                        f"/snap/{sdk_snap}/current/usr/lib/$CRAFT_ARCH_TRIPLET"
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
                        ],
                    ),
                },
            ],
        }

    @overrides
    def get_parts_snippet(self) -> Dict[str, Any]:
        source = get_extensions_data_dir() / "desktop" / "command-chain"
        base = self.yaml_data["base"]
        sdk_snap = _SDK_SNAP[base]

        return {
            "gnome/sdk": {
                "source": str(source),
                "plugin": "make",
                "build-snaps": [sdk_snap],
            }
        }
