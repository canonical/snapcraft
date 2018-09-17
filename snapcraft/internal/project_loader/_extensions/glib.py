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
from ._extension import Extension


class GlibExtensionClassicConfinementError(errors.SnapcraftError):
    fmt = "The glib extension doesn't support classic confinement."


class GlibExtension(Extension):
    """The GLib extension.

    This extension is to be used by applications that require GLib, but not all of GTK+.
    Examples might include games, or console applications.

    Note that this extension does not support classically-confined snaps at this time.
    """

    supported_bases = ("core16", "core18")

    def __init__(self, yaml_data: Dict[str, Any]) -> None:
        """Create a new GlibExtension.

        Note that this extension does not support classic snaps.

        :param dict yaml_data: Loaded snapcraft.yaml data.
        """

        super().__init__(yaml_data)

        if yaml_data.get("confinement") == "classic":
            raise GlibExtensionClassicConfinementError()

        # Add the following snippet to each app that uses the glib extension
        self.app_snippet = {
            "command-chain": [
                "snap/command-chain/desktop-init",
                "snap/command-chain/desktop-exports",
                "snap/command-chain/desktop-mark-and-exec",
            ]
        }

        # Add the following snippet to each part in the snapcraft.yaml
        self.part_snippet = {"after": ["glib-extension-init"]}

        # Add the following part
        self.parts = {
            "glib-extension": {
                "plugin": "dump",
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/desktop-common",
                "organize": {"desktop-*": "snap/command-chain/"},
                "build-packages": ["libglib2.0-dev"],
                "stage-packages": ["libglib2.0-0"],
            }
        }
