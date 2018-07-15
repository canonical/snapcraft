# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Tim Süberkrüb
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

from testtools.matchers import Equals

from snapcraft.internal import sources
from tests import unit


class Test7z(unit.TestCase):

    _7z_test_files = {"test1.txt", "test2.txt", "test3.txt"}

    def setUp(self):
        super().setUp()
        test_7z_file = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "7z", "test.7z"
        )
        self.test_7z_file_path = os.path.join(self.path, "test.7z")
        shutil.copy2(test_7z_file, self.test_7z_file_path)

    def test_pull_7z_file_must_extract(self):
        dest_dir = "src"
        os.makedirs(dest_dir)

        seven_zip_source = sources.SevenZip(self.test_7z_file_path, dest_dir)
        seven_zip_source.pull()

        self.assertThat(set(os.listdir(dest_dir)), Equals(self._7z_test_files))

    def test_extract_and_keep_7zfile(self):
        dest_dir = "src"
        os.makedirs(dest_dir)

        seven_zip_source = sources.SevenZip(self.test_7z_file_path, dest_dir)
        # This is the first step done by pull. We don't call pull to call the
        # second step with a different keep_7z value.
        shutil.copy2(seven_zip_source.source, seven_zip_source.source_dir)
        seven_zip_source.provision(dst=dest_dir, keep_7z=True)

        test_output_files = self._7z_test_files.union(
            {os.path.basename(self.test_7z_file_path)}
        )
        self.assertCountEqual(set(os.listdir(dest_dir)), test_output_files)

    def test_has_source_handler_entry(self):
        self.assertTrue(sources._source_handler["7z"] is sources.SevenZip)
