# -*- Mode:Python; indent-tabs-buildnil; tab-width:4 -*-
#
# Copyright (C) 2017-2019 Canonical Ltd
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
from typing import List, Optional, Type

from mypy_extensions import TypedDict

from snapcraft import yaml_utils
from snapcraft.internal.states._state import State

StateDict = TypedDict(
    "StateDict",
    {
        "build-packages": List[str],
        "build-snaps": List[str],
        "required-grade": Optional[str],
    },
)


class GlobalState(State):

    yaml_tag = u"!GlobalState"

    @classmethod
    def load(cls: Type["GlobalState"], *, filepath: str) -> "GlobalState":
        with open(filepath) as state_file:
            return yaml_utils.load(state_file)

    def save(self, *, filepath: str) -> None:
        dirpath = os.path.dirname(filepath)
        if dirpath:
            os.makedirs(dirpath, exist_ok=True)
        with open(filepath, "w") as state_file:
            yaml_utils.dump(self, stream=state_file)

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

    def get_required_grade(self) -> Optional[str]:
        return self.assets.get("required-grade")

    def set_required_grade(self, required_grade: str) -> None:
        self.assets["required-grade"] = required_grade

    def __init__(self, *, assets: Optional[StateDict] = None) -> None:
        super().__init__()
        if assets is None:
            self.assets: StateDict = {
                "build-packages": [],
                "build-snaps": [],
                "required-grade": None,
            }
        else:
            self.assets = assets
