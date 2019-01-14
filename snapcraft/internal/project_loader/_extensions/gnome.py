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

        platform_snap = "gnome-3-26-1604"  # default
        base = yaml_data.get("base")
        after_dependencies = []  # type: List[str]
        dependency_part = {}  # type: Dict[str, Any]
        if base is not None:
            if base == "core16":
                platform_snap = "gnome-3-26-1604"
                after_dependencies = ["gnome-extension-platform-dependencies"]
                dependency_part = {
                    "gnome-extension-platform-dependencies": {
                        "plugin": "nil",
                        "override-pull": """
                            add-apt-repository ppa:ubuntu-desktop/gnome-3-26
                            apt-get update
                            apt-get upgrade -yqq
                            """,
                    }
                }
            elif base == "core18":
                platform_snap = "gnome-3-28-1804"

        layout = {
            "layout": {
                "/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0": {
                    "bind": "$SNAP/gnome-platform/usr/lib/$SNAPCRAFT_ARCH_TRIPLET/webkit2gtk-4.0"
                }
            }
        }
        # Use passthrough when base: is not specified to work with old style snapcraft
        if base is None:
            layout = {"passthrough": {**layout}}

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
        exec_command = command_chain.pop()

        command_chain = command_chain + [
            "snap/command-chain/desktop-gnome-specific",
            exec_command,
        ]

        self.app_snippet = {
            **self.app_snippet,
            "command-chain": command_chain,
            "adapter": "full",
        }

        self.parts = {
            **self.parts,
            **dependency_part,
            "gnome-extension": {
                "after": after_dependencies,
                "plugin": "dump",
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/gnome",
                "source-type": "local",
                "organize": {"desktop-*": "snap/command-chain/"},
                "build-packages": ["build-essential", "libgtk-3-dev"],
            },
        }
