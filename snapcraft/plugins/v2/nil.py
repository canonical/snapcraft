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

from typing import Any, Dict, List, Set

from snapcraft.plugins.v2 import PluginV2


class NilPlugin(PluginV2):
    @classmethod
    def get_schema(cls) -> Dict[str, Any]:
        """Return a jsonschema compatible dictionary for the plugin properties."""
        return {
            "$schema": "http://json-schema.org/draft-04/schema#",
            "type": "object",
            "additionalProperties": False,
            "properties": {},
        }

    def get_build_packages(self) -> Set[str]:
        """
        Return a set of required packages to install in the build environment.
        """
        return set()

    def get_build_environment(self) -> List[Dict[str, str]]:
        """
        Return a dictionary with the environment to use in the build step.

        For consistency, the keys should be defined as SNAPCRAFT_<PLUGIN>_<KEY>.

        This method is called by the PluginHandler during the "build" step.
        """
        return list()

    def get_build_commands(self) -> List[str]:
        """
        Return a list of commands to run during the build step.

        This method is called by the PluginHandler during the "build" step.
        These commands are run in a single shell instance. This means
        that commands run before do affect the commands that follow.

        snapcraftctl can be used in the script to call out to snapcraft
        specific functionality.
        """
        return list()
