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

import os

from . import errors


def get_snapcraft_yaml(base_dir=None):
    possible_yamls = [
        os.path.join("snap", "snapcraft.yaml"),
        "snapcraft.yaml",
        ".snapcraft.yaml",
    ]

    if base_dir:
        possible_yamls = [os.path.join(base_dir, x) for x in possible_yamls]

    snapcraft_yamls = [y for y in possible_yamls if os.path.exists(y)]

    if not snapcraft_yamls:
        raise errors.MissingSnapcraftYamlError(
            snapcraft_yaml_file_path="snap/snapcraft.yaml"
        )
    elif len(snapcraft_yamls) > 1:
        raise errors.DuplicateSnapcraftYamlError(
            snapcraft_yaml_file_path=snapcraft_yamls[0],
            other_snapcraft_yaml_file_path=snapcraft_yamls[1],
        )

    return snapcraft_yamls[0]
