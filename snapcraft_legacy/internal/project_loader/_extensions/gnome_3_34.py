# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2019 Canonical Ltd
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

# Import types and tell flake8 to ignore the "unused" List.

from typing import Any, Dict, Tuple

from ._extension import Extension

_PLATFORM_SNAP = dict(core18="gnome-3-34-1804")


class ExtensionImpl(Extension):
    """This extension eases creation of snaps that integrate with GNOME 3.34.

    At build time it ensures the right build dependencies are setup and for
    the runtime it ensures the application is run in an environment catered
    for GNOME applications.

    It configures each application with the following plugs:

    \b
    - GTK3 Themes.
    - Common Icon Themes.
    - Common Sound Themes.
    - The GNOME runtime libraries and utilities corresponding to 3.34.

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
        return ("core18",)

    @staticmethod
    def get_supported_confinement() -> Tuple[str, ...]:
        return ("strict", "devmode")

    def __init__(self, *, extension_name: str, yaml_data: Dict[str, Any]) -> None:
        super().__init__(extension_name=extension_name, yaml_data=yaml_data)

        base = yaml_data["base"]
        platform_snap = _PLATFORM_SNAP[base]
        self.root_snippet = {
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
                    "command-chain": ["snap/command-chain/hooks-configure-desktop"],
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

        self.app_snippet = {
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

        self.part_snippet = {
            "build-environment": [
                {"PATH": "/snap/gnome-3-34-1804-sdk/current/usr/bin:$PATH"},
                {
                    "XDG_DATA_DIRS": "$SNAPCRAFT_STAGE/usr/share:/snap/gnome-3-34-1804-sdk/current/usr/share:/usr/share:$XDG_DATA_DIRS"
                },
                {
                    "LD_LIBRARY_PATH": "/snap/gnome-3-34-1804-sdk/current/lib/$SNAPCRAFT_ARCH_TRIPLET:/snap/gnome-3-34-1804-sdk/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET:/snap/gnome-3-34-1804-sdk/current/usr/lib:/snap/gnome-3-34-1804-sdk/current/usr/lib/vala-current:$LD_LIBRARY_PATH"
                },
                {
                    "PKG_CONFIG_PATH": "/snap/gnome-3-34-1804-sdk/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/pkgconfig:/snap/gnome-3-34-1804-sdk/current/usr/lib/pkgconfig:/snap/gnome-3-34-1804-sdk/current/usr/share/pkgconfig:$PKG_CONFIG_PATH"
                },
                {
                    "GETTEXTDATADIRS": "/snap/gnome-3-34-1804-sdk/current/usr/share/gettext-current:$GETTEXTDATADIRS"
                },
                {
                    "GDK_PIXBUF_MODULE_FILE": "/snap/gnome-3-34-1804-sdk/current/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/gdk-pixbuf-current/loaders.cache"
                },
                {
                    "PYTHONPATH": "/snap/gnome-3-34-1804-sdk/current/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
                },
            ]
        }

        self.parts = {
            "gnome-3-34-extension": {
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/desktop",
                "source-subdir": "gnome",
                "plugin": "make",
                "make-parameters": [
                    "WITH_BINDTEXTDOMAIN=1",
                    "PLATFORM_PLUG={plug}".format(plug=platform_snap),
                ],
                "build-snaps": ["gnome-3-34-1804-sdk/latest/stable"],
                "build-packages": ["gcc"],
            }
        }
