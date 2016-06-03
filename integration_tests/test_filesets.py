# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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

import integration_tests

from testtools.matchers import (
    DirExists,
    FileContains,
    Not,
)


class FilesetsTestCase(integration_tests.TestCase):

    def test_filesets(self):
        project_dir = 'simple-make-filesets'
        self.run_snapcraft('snap', project_dir)

        expected_dirs = (
            os.path.join('stage', 'new', 'dir1'),
            os.path.join('stage', 'new', 'dir2'),
            os.path.join('prime', 'new', 'dir2'),
        )
        for expected_dir in expected_dirs:
            self.assertThat(
                os.path.join(project_dir, expected_dir),
                DirExists())

        self.assertThat(
            os.path.join(project_dir, 'prime', 'new', 'dir1'),
            Not(DirExists()))

        expected_files = (
            (os.path.join('stage', 'share', 'file1'), 'file1\n'),
            (os.path.join('stage', 'share', 'file2'), 'file2\n'),
            (os.path.join('stage', 'new', 'dir2', 'file1'), 'file1\n'),
            (os.path.join('prime', 'share', 'file1'), 'file1\n'),
            (os.path.join('prime', 'share', 'file2'), 'file2\n'),
            (os.path.join('prime', 'new', 'dir2', 'file1'), 'file1\n'),
        )
        for path, content in expected_files:
            self.assertThat(
                os.path.join(project_dir, path),
                FileContains(content))
