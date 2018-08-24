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

from typing import Any, Dict, Set


class Template:
    @classmethod
    def supported_bases(cls) -> Set[str]:
        """Bases that are supported by this template.

        :returns: Set of base names supported by this template.
        :rtype: set
        """
        return set()

    def __init__(self, yaml_data: Dict[str, Any]) -> None:
        """Create a new Template.

        :param dict yaml_data: Loaded snapcraft.yaml data.
        """
        pass

    def app_snippet(self) -> Dict[str, Any]:
        """App snippet for this template.

        :returns: Dict of properties to apply to apps using this template.
        :rtype: dict
        """
        return dict()

    def part_snippet(self) -> Dict[str, Any]:
        """Part snippet for this template.

        :returns: Dict of properties to apply to parts using this template.
        :rtype: dict
        """
        return dict()

    def parts(self) -> Dict[str, Any]:
        """Parts definitions required for this template.

        :returns: Dict of part definitions required for this part.
        :rtype: dict
        """
        return dict()
