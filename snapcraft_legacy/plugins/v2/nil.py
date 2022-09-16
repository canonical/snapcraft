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

"""The nil plugin is useful in two contexts.

First, it can be used for parts that identify no source, and can
be defined purely by using properties automatically included
by Snapcraft, such as "stage-packages".

The second use is for parts that do define a source (which will be
fetched), but for which the build step then needs to be explicitly
defined using "override-build"; otherwise, even though the source is
fetched, while it is copied to the build/ directory, nothing will be
further copied to the install/ directory.

In short, for the case of a part that uses the nil plugin and defines
a "source", it is up to the developer to then define the "override-build"
step that, in some way, populates the $SNAPCRAFT_PART_INSTALL or
(core22) $CRAFT_PART_INSTALL directory.

"""

from typing import Any, Dict, List, Set

from snapcraft_legacy.plugins.v2 import PluginV2


class NilPlugin(PluginV2):
    @classmethod
    def get_schema(cls) -> Dict[str, Any]:
        return {
            "$schema": "http://json-schema.org/draft-04/schema#",
            "type": "object",
            "additionalProperties": False,
            "properties": {},
        }

    def get_build_snaps(self) -> Set[str]:
        return set()

    def get_build_packages(self) -> Set[str]:
        return set()

    def get_build_environment(self) -> Dict[str, str]:
        return dict()

    def get_build_commands(self) -> List[str]:
        return list()
