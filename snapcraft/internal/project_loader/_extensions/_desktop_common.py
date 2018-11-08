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

        self.root_snippet = {"assumes": ["command-chain"]}

        self.app_snippet = {
            "command-chain": [
                "snap/command-chain/desktop-init",
                "snap/command-chain/desktop-exports",
                "snap/command-chain/desktop-mark-and-exec",
            ]
        }

        self.parts = {
            "desktop-common-extension": {
                "plugin": "dump",
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/desktop-common",
                "organize": {"desktop-*": "snap/command-chain/"},
            }
        }
