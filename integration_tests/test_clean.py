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

from testtools.matchers import (
    DirExists,
    Not
)

import integration_tests


class CleanTestCase(integration_tests.TestCase):

    def test_clean(self):
        project_dir = 'simple-make'
        self.run_snapcraft('snap', project_dir)

        snap_dirs = ('stage', 'parts', 'prime')
        for dir_ in snap_dirs:
            self.assertThat(
                os.path.join(project_dir, dir_), DirExists())

        self.run_snapcraft('clean', project_dir)
        for dir_ in snap_dirs:
            self.assertThat(
                os.path.join(project_dir, dir_), Not(DirExists()))

    def test_clean_again(self):
        # Clean a second time doesn't fail.
        # Regression test for https://bugs.launchpad.net/snapcraft/+bug/1497371
        project_dir = 'simple-make'
        self.run_snapcraft('snap', project_dir)
        self.run_snapcraft('clean', project_dir)
        self.run_snapcraft('clean', project_dir)

    # Regression test for LP: #1596596
    def test_clean_invalid_yaml(self):
        project_dir = 'invalid-snap'
        self.run_snapcraft('clean', project_dir)
        self.assertThat(os.path.join(project_dir, 'parts'), Not(DirExists()))
        self.assertThat(os.path.join(project_dir, 'stage'), Not(DirExists()))
        self.assertThat(os.path.join(project_dir, 'prime'), Not(DirExists()))
