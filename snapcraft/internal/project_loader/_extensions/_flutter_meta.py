# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

import textwrap
from typing import Any, Dict, Optional, Tuple

from .. import errors

_PLATFORM_SNAP = dict(core18="gnome-3-28-1804")
_DOC_TEMPLATE = textwrap.dedent(
    """\
   This extension eases creation of snaps that use Flutter

    It tracks the {channel} channel for flutter.

    At build time it ensures the right build dependencies are setup and for
    the runtime it ensures the application is run in an environment catered
    for Flutter applications.

    It configures each application with the following plugs:

    \b
    - GTK3 Themes.
    - Common Icon Themes.
    - Common Sound Themes.
    - The GNOME runtime libraries and utilities corresponding to 3.28.

    For easier desktop integration, it also configures each application
    entry with these additional plugs:

    \b
    - desktop (https://snapcraft.io/docs/desktop-interface)
    - desktop-legacy (https://snapcraft.io/docs/desktop-legacy-interface)
    - gsettings (https://snapcraft.io/docs/gsettings-interface)
    - wayland (https://snapcraft.io/docs/wayland-interface)
    - opengl (https://snapcraft.io/docs/opengl-interface)
    - x11 (https://snapcraft.io/docs/x11-interface)
"""
)


def get_platform_snap_for_base(self, base: str) -> str:
    return _PLATFORM_SNAP[base]


class FlutterMetaExtension(type):
    def __new__(cls, name, bases, attrs):
        # Not setting these is a development error.
        channel = attrs["channel"]

        def is_experimental(base: Optional[str]) -> bool:
            return False

        def get_supported_bases() -> Tuple[str, ...]:
            return attrs["supported_bases"]

        def get_supported_confinement() -> Tuple[str, ...]:
            return attrs["supported_confinement"]

        def __repr__(self) -> str:
            return f"<Flutter Extension for {channel!r}>"

        def __init__(self, *, extension_name: str, yaml_data: Dict[str, Any]) -> None:
            base: Optional[str] = yaml_data.get("base")
            # A base is required in order to use extensions, so raise an error if not specified.
            if base is None:
                raise errors.ExtensionBaseRequiredError()

            if base not in self.get_supported_bases():
                raise errors.ExtensionUnsupportedBaseError(extension_name, base)

            # Default to devmode if confinement is not set.
            confinement = yaml_data.get("confinement", "devmode")
            if confinement not in self.get_supported_confinement():
                raise errors.ExtensionUnsupportedConfinementError(
                    extension_name, confinement
                )

            platform_snap = _PLATFORM_SNAP[base]
            self.root_snippet["plugs"].update(
                {
                    platform_snap: {
                        "interface": "content",
                        "target": "$SNAP/gnome-platform",
                        "default-provider": "{snap}".format(snap=platform_snap),
                    }
                }
            )

        x = super().__new__(
            cls,
            name,
            bases,
            {
                get_supported_bases.__name__: staticmethod(get_supported_bases),
                get_supported_confinement.__name__: staticmethod(
                    get_supported_confinement
                ),
                is_experimental.__name__: staticmethod(is_experimental),
                __repr__.__name__: __repr__,
                __init__.__name__: __init__,
            },
        )

        x.__doc__ = _DOC_TEMPLATE.format(channel=channel)
        x.root_snippet = {
            "plugs": {
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
            },
            "environment": {"SNAP_DESKTOP_RUNTIME": "$SNAP/gnome-platform"},
            "layout": {
                "/usr/share/xml/iso-codes": {
                    "bind": "$SNAP/gnome-platform/usr/share/xml/iso-codes"
                }
            },
        }

        x.app_snippet = {
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

        x.part_snippet = dict()

        x.parts = {
            "gnome-3-28-extension": {
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/desktop",
                "source-subdir": "gnome",
                "plugin": "make",
                "build-packages": ["gcc", "libgtk-3-dev"],
            },
            "flutter-extension": {
                "plugin": "nil",
                "override-pull": textwrap.dedent(
                    f"""\
                    flutter channel {attrs["channel"]}
                    flutter config --enable-linux-desktop
                    flutter upgrade
                    flutter doctor
                    """
                ),
                "build-snaps": ["flutter/latest/stable"],
            },
        }

        return x
