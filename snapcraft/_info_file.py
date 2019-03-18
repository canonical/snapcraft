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

import os

from typing import Any
from snapcraft import yaml_utils


class InfoFile(dict):
    """A helper to persist dictionaries as yaml files."""

    def __init__(self, path: str) -> None:
        self._path = path

    def load(self):
        if os.path.exists(self._path):
            with open(self._path) as info_file:
                self.clear()
                self.update(yaml_utils.load(info_file))

    def save(self, **data: Any) -> None:
        dirpath = os.path.dirname(self._path)
        if dirpath:
            os.makedirs(dirpath, exist_ok=True)
        with open(self._path, "w") as info_file:
            data.update(dict(self))
            yaml_utils.dump(data, stream=info_file)
