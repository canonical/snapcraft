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
from typing import Dict, List, Type

import yaml

from snapcraft.internal.states._state import State


class _GlobalStateLoader(yaml.Loader):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        self.add_constructor(u"!GlobalState", type(self).construct_global_state)

    def construct_global_state(self, node) -> "GlobalState":
        parameters = self.construct_mapping(node)
        return GlobalState(**parameters)


class GlobalState(State):

    yaml_tag = u"!GlobalState"

    @classmethod
    def load(cls: Type["GlobalState"], *, filepath: str) -> "GlobalState":
        with open(filepath) as state_file:
            return yaml.load(state_file, _GlobalStateLoader)

    def save(self, *, filepath: str) -> None:
        dirpath = os.path.dirname(filepath)
        if dirpath:
            os.makedirs(dirpath, exist_ok=True)
        with open(filepath, "w") as state_file:
            yaml.dump(self, stream=state_file)

    def get_build_packages(self) -> List[str]:
        return self.assets.get("build-packages", [])

    def append_build_packages(self, build_packages: List[str]) -> None:
        current_build_packages = self.get_build_packages()
        new_packages = [b for b in build_packages if b not in current_build_packages]
        self.assets["build-packages"] = current_build_packages + new_packages

    def get_build_snaps(self) -> List[str]:
        return self.assets.get("build-snaps", [])

    def append_build_snaps(self, build_snaps: List[str]) -> None:
        current_build_snaps = self.get_build_snaps()
        new_snaps = [b for b in build_snaps if b not in current_build_snaps]
        self.assets["build-snaps"] = current_build_snaps + new_snaps

    def __init__(self, *, assets: Dict[str, List[str]] = None) -> None:
        super().__init__()
        if assets is None:
            self.assets = dict()  # type: Dict[str, List[str]]
        else:
            self.assets = assets
