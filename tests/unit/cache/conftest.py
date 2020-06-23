# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
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

import pytest

from snapcraft.internal import cache


@pytest.fixture()
def random_data_file(tmp_path):
    """Return a file with some text in it."""
    file_path = tmp_path / "to_hash"
    with file_path.open("w") as to_hash_file:
        print("random stub data", file=to_hash_file, end="")

    return file_path.as_posix()


@pytest.fixture()
def file_cache(xdg_dirs):
    """Return a FileCache instance."""
    return cache.FileCache()
