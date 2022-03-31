# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2022 Canonical Ltd
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

from typing import Any, Dict, Optional, Tuple

from .extension import Extension, get_extensions_data_dir

_PLATFORM_SNAP = dict(core22="gnome-42-2204")
_SDK_SNAP = dict(core22="gnome-42-2204-sdk")


class GNOME(Extension):
    """This extension eases creation of snaps that integrate with GNOME.

    When used with core22 GNOME 42 will be used.

    At build time it ensures the right build dependencies are setup and for
    the runtime it ensures the application is run in an environment catered
    for GNOME applications.

    It configures each application with the following plugs:

    \b
    - GTK3 Themes.
    - Common Icon Themes.
    - Common Sound Themes.
    - The GNOME runtime libraries and utilities corresponding to 3.38.

    For easier desktop integration, it also configures each application
    entry with these additional plugs:

    \b
    - desktop (https://snapcraft.io/docs/desktop-interface)
    - desktop-legacy (https://snapcraft.io/docs/desktop-legacy-interface)
    - gsettings (https://snapcraft.io/docs/gsettings-interface)
    - opengl (https://snapcraft.io/docs/opengl-interface)
    - wayland (https://snapcraft.io/docs/wayland-interface)
    - x11 (https://snapcraft.io/docs/x11-interface)
    """

    @staticmethod
    def get_supported_bases() -> Tuple[str, ...]:
        return ("core22",)

    @staticmethod
    def get_supported_confinement() -> Tuple[str, ...]:
        return ("strict", "devmode")

    @staticmethod
    def is_experimental(base: Optional[str]) -> bool:
        return False

    def get_app_snippet(self):
        return {
            # TODO: command-chain to check connection.
            "command-chain": ["$SNAP/gnome-platform/desktop-launch"],
            "plugs": [
                "desktop",
                "desktop-legacy",
                "gsettings",
                "opengl",
                "wayland",
                "x11",
            ],
        }

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
                    "default-provider": "{snap}".format(snap=platform_snap),
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

    def get_part_snippet(self) -> Dict[str, Any]:
        base = self.yaml_data["base"]
        sdk_snap = _SDK_SNAP[base]

        return {
            "build-environment": [
                {"PATH": f"/snap/{sdk_snap}/current/usr/bin:$PATH"},
                {
                    "XDG_DATA_DIRS": f"$SNAPCRAFT_STAGE/usr/share:/snap/{sdk_snap}/current/usr/share:/usr/share:$XDG_DATA_DIRS"
                },
                {
                    "LD_LIBRARY_PATH": ":".join(
                        [
                            f"/snap/{sdk_snap}/current/lib/$SNAPCRAFT_ARCH_TRIPLET",
                            f"/snap/{sdk_snap}/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET",
                            f"/snap/{sdk_snap}/current/usr/lib",
                            f"/snap/{sdk_snap}/current/usr/lib/vala-current",
                            f"/snap/{sdk_snap}/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/pulseaudio",
                        ]
                    )
                    + "${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
                },
                {
                    "PKG_CONFIG_PATH": f"/snap/{sdk_snap}/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/pkgconfig:/snap/{sdk_snap}/current/usr/lib/pkgconfig:/snap/{sdk_snap}/current/usr/share/pkgconfig:$PKG_CONFIG_PATH"
                },
                {
                    "GETTEXTDATADIRS": f"/snap/{sdk_snap}/current/usr/share/gettext-current:$GETTEXTDATADIRS"
                },
                {
                    "GDK_PIXBUF_MODULE_FILE": f"/snap/{sdk_snap}/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/gdk-pixbuf-current/loaders.cache"
                },
                {
                    "ACLOCAL_PATH": f"/snap/{sdk_snap}/current/usr/share/aclocal${{ACLOCAL_PATH:+:$ACLOCAL_PATH}}"
                },
                {
                    "PYTHONPATH": ":".join(
                        [
                            f"/snap/{sdk_snap}/current/usr/lib/python3.10",
                            f"/snap/{sdk_snap}/current/usr/lib/python3/dist-packages",
                        ]
                    )
                    + "${PYTHONPATH:+:$PYTHONPATH}"
                },
            ]
        }

    def get_parts_snippet(self) -> Dict[str, Any]:
        source = get_extensions_data_dir() / "fonts"
        return {
            "gnome/hook-configure-fonts": {
                "source": str(source),
                "plugin": "nil",
                "override-build": (
                    "install -D -m 0755 hooks-configure-fonts "
                    "${SNAPCRAFT_PART_INSTALL}/snap/command-chain/",
                ),
            }
        }
