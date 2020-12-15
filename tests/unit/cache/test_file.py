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

import os
import shutil

from snapcraft.file_utils import calculate_hash


class TestFileCache:

    scenarios = [
        ("sha384", dict(algo="sha384")),
        ("md5", dict(algo="md5")),
        ("sha1", dict(algo="sha1")),
        ("sha224", dict(algo="sha224")),
        ("sha256", dict(algo="sha256")),
        ("sha384", dict(algo="sha384")),
        ("sha512", dict(algo="sha512")),
        ("sha3_256", dict(algo="sha3_256")),
        ("sha3_384", dict(algo="sha3_384")),
        ("sha3_512", dict(algo="sha3_512")),
    ]

    def test_get_nothing_cached(self, file_cache, algo):
        assert file_cache.get(algorithm=algo, hash="1") is None

    def test_cache_and_retrieve(self, random_data_file, file_cache, algo):
        calculated_hash = calculate_hash(random_data_file, algorithm=algo)
        cached_file = file_cache.cache(
            filename=random_data_file, algorithm=algo, hash=calculated_hash
        )
        leaf_path = os.path.join(algo, calculated_hash)
        assert cached_file.endswith(leaf_path)

        retrieved_file = file_cache.get(algorithm=algo, hash=calculated_hash)
        assert retrieved_file.endswith(leaf_path)

    def test_cache_not_possible(self, random_data_file, file_cache, algo):
        bad_calculated_hash = "1"

        cached_file = file_cache.cache(
            filename=random_data_file, algorithm=algo, hash=bad_calculated_hash
        )
        assert cached_file is None

    def test_cache_file_copy_error(
        self, monkeypatch, random_data_file, file_cache, algo
    ):
        calculated_hash = calculate_hash(random_data_file, algorithm=algo)

        def fake_copy(*args, **kwargs):
            raise OSError()

        monkeypatch.setattr(shutil, "copyfile", fake_copy)

        cached_file = file_cache.cache(
            filename=random_data_file, algorithm=algo, hash=calculated_hash
        )

        assert cached_file is None
