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
from typing import List  # noqa: F401

from ._extension import Extension


class DesktopCommonExtension(Extension):
    """The Common Desktop extension.
    This extension is to be subclassed by the gnome and kde extensions.
    It includes common code that each desktop extension would otherwise
    need to duplicate.
    """

    def __init__(self, yaml_data: Dict[str, Any]) -> None:
        """Create a new DesktopCommonExtension.
        :param dict yaml_data: Loaded snapcraft.yaml data.
        """

        super().__init__(yaml_data)

        # Check if we've been already applied
        parts = yaml_data.get("parts")
        if parts and "desktop-common-extension" in parts.keys():
            self.plugs = {}  # type: Dict[str, Any]
            self.environment = {}  # type: Dict[str, str]
            self.command_chain = []  # type: List[str]
            self.parts = {}  # type: Dict[str, Any]
            return

        self.plugs = {
            "gtk-3-themes": {
                "interface": "content",
                "target": "$SNAP/data-dir/themes",
                "default-provider": "gtk-common-themes:gtk-3-themes",
            },
            "icon-themes": {
                "interface": "content",
                "target": "$SNAP/data-dir/icons",
                "default-provider": "gtk-common-themes:icon-themes",
            },
            "sound-themes": {
                "interface": "content",
                "target": "$SNAP/data-dir/sounds",
                "default-provider": "gtk-common-themes:sound-themes",
            },
        }

        self.environment = {}  # type: Dict[str, str]

        self.root_snippet = {
            "assumes": ["command-chain"],
            "environment": self.environment,
            "plugs": self.plugs,
        }

        self.command_chain = [
            "snap/command-chain/desktop-init",
            "snap/command-chain/desktop-common",
        ]

        self.app_snippet = {"command-chain": self.command_chain}

        self.parts = {
            "desktop-common-extension": {
                "plugin": "dump",
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/desktop-common",
                "organize": {"desktop-*": "snap/command-chain/"},
            },
            "desktop-common-bindtextdomain": {
                "plugin": "make",
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/bindtextdomain",
            },
        }
