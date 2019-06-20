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

# Import types and tell flake8 to ignore the "unused" List.
from typing import Any, Dict, List  # noqa: F401

from ._desktop_common import DesktopCommonExtension
from .. import errors


class Gnome_3_28Extension(DesktopCommonExtension):
    """The Gnome extension.
    This extension is to be used by applications that require GTK+.
    Examples might include productivity applications or utilities.
    Note that this extension does not support classically-confined snaps at this time.
    """

    supported_bases = ("core18",)
    supports_classic = False

    def __init__(self, yaml_data: Dict[str, Any]) -> None:
        """Create a new GnomeExtension.
        Note that this extension does not support classic snaps.
        :param dict yaml_data: Loaded snapcraft.yaml data.
        """

        super().__init__(yaml_data)

        platform_snap = ""
        base = yaml_data.get("base")

        if base == "core18":
            platform_snap = "gnome-3-28-1804"
        else:
            raise errors.ExtensionUnsupportedBaseError("gnome", base)

        layout = {
            "layout": {
                "/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0": {
                    "bind": "$SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0"
                },
                "/usr/share/xml/iso-codes": {
                    "bind": "$SNAP/gnome-platform/usr/share/xml/iso-codes"
                },
            }
        }  # type: Dict[str, Any]

        self.root_snippet = {
            **self.root_snippet,
            **layout,
            "plugs": {
                **self.plugs,
                platform_snap: {
                    "interface": "content",
                    "target": "$SNAP/gnome-platform",
                    "default-provider": "{snap}:{slot}".format(
                        snap=platform_snap, slot=platform_snap
                    ),
                },
            },
            "environment": {
                **self.environment,
                "SNAP_DESKTOP_RUNTIME": "$SNAP/gnome-platform",
            },
        }

        command_chain = self.app_snippet["command-chain"]

        command_chain = command_chain + ["snap/command-chain/desktop-gnome-specific"]

        self.app_snippet = {
            **self.app_snippet,
            "command-chain": command_chain,
        }

        self.parts = {
            **self.parts,
            "gnome-extension": {
                "plugin": "dump",
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/gnome",
                "source-type": "local",
                "organize": {"desktop-*": "snap/command-chain/"},
                "build-packages": ["libgtk-3-dev"],
            },
        }
