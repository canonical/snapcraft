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

from typing import List, Set

from snapcraft.plugins.v2 import nil


class PluginImpl(nil.NilPlugin):
    @classmethod
    def schema(cls):
        schema = super().schema()
        schema["properties"]["foo"] = {"type": "string"}
        return schema

    def get_build_packages(self) -> Set[str]:
        build_packages = super().get_build_packages()
        build_packages.add("gcc")
        return build_packages

    def get_build_commands(self) -> List[str]:
        commands = super().get_build_commands()
        commands.append("mkdir -p ${SNAPCRAFT_PART_INSTALL}/bin")
        commands.append("gcc hello.c -o ${SNAPCRAFT_PART_INSTALL}/bin/hello")
        return commands
