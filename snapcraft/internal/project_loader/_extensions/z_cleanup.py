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


_CLEAN_SCRIPT = """
set -eux
for snap in {snaps}; do
    cd "/snap/$snap/current" && find . -type f,l -exec rm -f "$SNAPCRAFT_PRIME/{{}}" \\;
done
"""


class ExtensionImpl(Extension):
    """This extension cleans libraries that are present in connected content snaps
    """

    @staticmethod
    def get_supported_bases() -> Tuple[str, ...]:
        return ("core18",)

    @staticmethod
    def get_supported_confinement() -> Tuple[str, ...]:
        return ("strict", "devmode")

    def __init__(self, *, extension_name: str, yaml_data: Dict[str, Any]) -> None:
        super().__init__(extension_name=extension_name, yaml_data=yaml_data)

        base: str = yaml_data["base"]

        parts = []
        snaps = [base]

        for part in yaml_data.get("parts", {}).keys():
            parts.append(part)

        for plug in yaml_data.get("plugs", {}).values():
            if type(plug) is dict:
                if plug.get("default-provider"):
                    snaps.append(plug.get("default-provider"))

        snaps = list(set(snaps))
        snaps.sort()
        self.parts = {
            "cleanup": {
                "after": parts,
                "plugin": "nil",
                "build-snaps": snaps,
                "override-prime": _CLEAN_SCRIPT.format(snaps=" ".join(snaps)),
            }
        }
