# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

from typing import Any, Dict

from snapcraft.internal import errors
from ._desktop_common import DesktopCommonExtension


class GnomeExtensionClassicConfinementError(errors.SnapcraftError):
    fmt = "The gnome extension doesn't support classic confinement."


class GnomeExtension(DesktopCommonExtension):
    """The Gnome extension.
    This extension is to be used by applications that require GTK+.
    Examples might include productivity applications or utilities.
    Note that this extension does not support classically-confined snaps at this time.
    """

    supported_bases = ("core16", "core18")

    def __init__(self, yaml_data: Dict[str, Any]) -> None:
        """Create a new GnomeExtension.
        Note that this extension does not support classic snaps.
        :param dict yaml_data: Loaded snapcraft.yaml data.
        """

        super().__init__(yaml_data)

        if yaml_data.get("confinement") == "classic":
            raise GnomeExtensionClassicConfinementError()

        # This extension utilizes command-chain; add the proper assumes so a snap using
        # this extension can't be installed on a snapd that doesn't support
        # command-chain.
        self.root_snippet = {
            **self.root_snippet,
            "plugs": {
                "gnome-extension-common-themes": {
                    "interface": "content",
                    "target": "$SNAP/data-dir/themes",
                    "default-provider": "gtk-common-themes:gtk-3-themes",
                },
                "gnome-extension-common-icons": {
                    "interface": "content",
                    "target": "$SNAP/data-dir/icons",
                    "default-provider": "gtk-common-themes:icon-themes",
                },
                "gnome-extension-common-sounds": {
                    "interface": "content",
                    "target": "$SNAP/data-dir/sounds",
                    "default-provider": "gtk-common-themes:sounds-themes",
                },
            },
        }

        command_chain = self.app_snippet["command-chain"]
        exec_command = command_chain.pop()

        command_chain = command_chain + [
            "snap/command-chain/desktop-gnome-specific",
            exec_command,
        ]

        # Add the following snippet to each app that uses the glib extension
        self.app_snippet = {**self.app_snippet, "command-chain": command_chain}

        # Add the following part
        self.parts = {
            **self.parts,
            "gnome-extension": {
                "plugin": "dump",
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/gnome",
                "source-type": "local",
                "organize": {
                    "desktop-*": "snap/command-chain/",
                    # The next line moves files into a location to allow GTK_EXE_DIR
                    # to be suitable to find the GTK files. This allows us to use GTK2
                    # and GTK3 together in the same snap.
                    # If you need GTK2 support, duplicate this organize line replacing
                    # gtk-3.0 with gtk-2.0 in an additional part ensuring that the part
                    # also pulls GTK2 libraries. It should then seamlessly work with
                    # this extension.
                    "usr/lib/$SNAPCRAFT_ARCH_TRIPLET/gtk-3.0": "usr/lib/gtk-3.0",
                },
                "build-packages": ["build-essential", "libgtk-3-dev"],
                "stage-packages": [
                    "libxkbcommon0",
                    "shared-mime-info",
                    "libgtk-3-0",
                    "libgdk-pixbuf2.0-0",
                    "libglib2.0-bin",
                    "libgtk-3-bin",
                    "unity-gtk3-module",
                    "libappindicator3-1",
                    "locales-all",
                    "xdg-user-dirs",
                    "ibus-gtk3",
                    "libibus-1.0-5",
                    "fcitx-frontend-gtk3",
                ],
            },
        }
