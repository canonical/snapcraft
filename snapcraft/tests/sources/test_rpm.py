# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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
import libarchive
import shutil

from snapcraft.internal import sources
from snapcraft import tests


class TestRpm(tests.TestCase):

    def test_pull_rpm_file_must_extract(self):
        rpm_file_name = 'test.rpm'
        dest_dir = 'src'
        os.makedirs(dest_dir)

        test_file_path = os.path.join(self.path, 'test.txt')
        open(test_file_path, 'w').close()
        rpm_file_path = os.path.join(self.path, rpm_file_name)
        os.chdir(self.path)
        with libarchive.file_writer(rpm_file_path, 'cpio', 'gzip') as rpm:
            rpm.add_files('test.txt')

        rpm_source = sources.Rpm(rpm_file_path, dest_dir)
        rpm_source.pull()

        self.assertEqual(os.listdir(dest_dir), ['test.txt'])

    def test_extract_and_keep_rpmfile(self):
        rpm_file_name = 'test.rpm'
        dest_dir = 'src'
        os.makedirs(dest_dir)

        test_file_path = os.path.join(self.path, 'test.txt')
        open(test_file_path, 'w').close()
        rpm_file_path = os.path.join(self.path, rpm_file_name)
        os.chdir(self.path)
        with libarchive.file_writer(rpm_file_path, 'cpio', 'gzip') as rpm:
            rpm.add_files('test.txt')

        rpm_source = sources.Rpm(rpm_file_path, dest_dir)
        # This is the first step done by pull. We don't call pull to call the
        # second step with a different keep_rpm value.
        shutil.copy2(rpm_source.source, rpm_source.source_dir)
        rpm_source.provision(dst=dest_dir, keep_rpm=True)

        test_output_files = ['test.txt', rpm_file_name]
        self.assertCountEqual(os.listdir(dest_dir), test_output_files)
