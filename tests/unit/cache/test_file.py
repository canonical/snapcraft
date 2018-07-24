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
from unittest.mock import patch

from testtools.matchers import EndsWith, Is

from snapcraft.file_utils import calculate_hash
from snapcraft.internal import cache
from tests import unit


class FileCacheTestCase(unit.TestCase):

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

    def setUp(self):
        super().setUp()
        self.file_cache = cache.FileCache()

    def test_get_nothing_cached(self):
        file = self.file_cache.get(algorithm=self.algo, hash="1")
        self.assertThat(file, Is(None))

    def test_cache_and_retrieve(self):
        with open("hash_file", "w") as f:
            f.write("random stub data")

        calculated_hash = calculate_hash("hash_file", algorithm=self.algo)
        file = self.file_cache.cache(
            filename="hash_file", algorithm=self.algo, hash=calculated_hash
        )
        leaf_path = os.path.join(self.algo, calculated_hash)
        self.assertThat(file, EndsWith(leaf_path))

        retrieved_file = self.file_cache.get(algorithm=self.algo, hash=calculated_hash)
        self.assertThat(retrieved_file, EndsWith(leaf_path))

    def test_cache_not_possible(self):
        with open("hash_file", "w") as f:
            f.write("random stub data")

        bad_calculated_hash = "1"
        file = self.file_cache.cache(
            filename="hash_file", algorithm=self.algo, hash=bad_calculated_hash
        )
        self.assertThat(file, Is(None))

    def test_cache_file_copy_error(self):
        with open("hash_file", "w") as f:
            f.write("random stub data")

        calculated_hash = calculate_hash("hash_file", algorithm=self.algo)
        with patch("shutil.copyfile") as mock_copyfile:
            mock_copyfile.side_effect = OSError()
            file = self.file_cache.cache(
                filename="hash_file", algorithm=self.algo, hash=calculated_hash
            )
        self.assertThat(file, Is(None))
