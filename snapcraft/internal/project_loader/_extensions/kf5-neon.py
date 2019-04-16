# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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


class Kf5_neonExtension(DesktopCommonExtension):
    """The KDE Frameworks 5 Neon extension.
    This extension is to be used by applications that use KDE Neon.
    Examples might include productivity applications or utilities.
    Note that this extension does not support classically-confined snaps
    at this time.
    """

    supported_bases = ("core18",)
    supports_classic = False

    def __init__(self, yaml_data: Dict[str, Any]) -> None:
        """Create a new Kde5Extension.
        Note that this extension does not support classic snaps.
        :param dict yaml_data: Loaded snapcraft.yaml data.
        """

        super().__init__(yaml_data)

        provider = ""
        content = ""
        base = yaml_data.get("base")

        if base == "core18":
            provider = "kde-frameworks-5-core18"
            content = "kde-frameworks-5-core18-all"
        else:
            raise errors.ExtensionUnsupportedBaseError("kf5-neon", base)

        self.root_snippet = {
            **self.root_snippet,
            "plugs": {
                **self.plugs,
                "kde-frameworks-5-plug": {
                    "interface": "content",
                    "content": content,
                    "target": "kf5",
                    "default-provider": provider,
                },
            },
            "environment": {**self.environment, "SNAP_DESKTOP_RUNTIME": "$SNAP/kf5"},
        }

        command_chain = self.app_snippet["command-chain"]
        command_chain = command_chain + ["snap/command-chain/kf5-launch"]

        self.app_snippet = {
            **self.app_snippet,
            "command-chain": command_chain,
            "adapter": "full",
        }

        self.parts = {
            **self.parts,
            "kde-frameworks-5-env": {
                "plugin": "dump",
                "source": "$SNAPCRAFT_EXTENSIONS_DIR/kf5-neon",
                "source-type": "local",
                "organize": {"kf5-launch": "snap/command-chain/"},
                "build-packages": ["build-essential", "cmake"],
            },
        }
