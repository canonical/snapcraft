# -*- Mode:Python; indent-tabs-buildnil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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

import os
from typing import List

import yaml

from snapcraft.internal.states._state import State


def _global_state_constructor(loader, node):
    parameters = loader.construct_mapping(node)
    return GlobalState(**parameters)


yaml.add_constructor(u"!GlobalState", _global_state_constructor)


class GlobalState(State):

    yaml_tag = u"!GlobalState"

    @classmethod
    def load(cls, *, filepath: str):
        with open(filepath) as state_file:
            return yaml.load(state_file)

    def save(self, *, filepath: str) -> None:
        dirpath = os.path.dirname(filepath)
        if dirpath:
            os.makedirs(dirpath, exist_ok=True)
        with open(filepath, "w") as state_file:
            yaml.dump(self, stream=state_file)

    def get_build_packages(self) -> List[str]:
        return self.assets.get("build-packages", [])

    def get_build_snaps(self) -> List[str]:
        return self.assets.get("build-snaps", [])

    def __init__(self, *, build_packages: List[str], build_snaps: List[str]) -> None:
        super().__init__()
        self.assets = {"build-packages": build_packages, "build-snaps": build_snaps}
