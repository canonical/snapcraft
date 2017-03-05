# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Tim Süberkrüb
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
import os.path
import shutil

from snapcraft.internal import sources
from snapcraft import tests


class Test7z(tests.TestCase):

    @staticmethod
    def relative_path():
        return os.path.dirname(os.path.realpath(__file__))

    def test_pull_7z_file_must_extract(self):
        test_7z_file_path = os.path.join(
            Test7z.relative_path(),
            '7z', 'test.7z'
        )

        dest_dir = 'src'
        os.makedirs(dest_dir)

        _7z_source = sources._7z(test_7z_file_path, dest_dir)
        _7z_source.pull()

        self.assertEqual(os.listdir(dest_dir), ['test.txt'])

    def test_extract_and_keep_7zfile(self):
        test_7z_file_name = 'test.7z'
        test_7z_file_path = os.path.join(
            Test7z.relative_path(),
            '7z',
            test_7z_file_name
        )
        dest_dir = 'src'
        os.makedirs(dest_dir)

        _7z_source = sources._7z(test_7z_file_path, dest_dir)
        # This is the first step done by pull. We don't call pull to call the
        # second step with a different keep_7z value.
        shutil.copy2(_7z_source.source, _7z_source.source_dir)
        _7z_source.provision(dst=dest_dir, keep_7z=True)

        test_output_files = ['test.txt', test_7z_file_name]
        self.assertCountEqual(os.listdir(dest_dir), test_output_files)
