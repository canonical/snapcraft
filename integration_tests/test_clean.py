# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016, 2017 Canonical Ltd
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

from testtools.matchers import (
    DirExists,
    Not
)

import integration_tests


class CleanTestCase(integration_tests.TestCase):

    def test_clean(self):
        self.copy_project_to_cwd('simple-make')
        self.run_snapcraft('snap')

        snap_dirs = ('stage', 'parts', 'prime')
        for dir_ in snap_dirs:
            self.assertThat(dir_, DirExists())

        self.run_snapcraft('clean')
        for dir_ in snap_dirs:
            self.assertThat(dir_, Not(DirExists()))

    def test_clean_again(self):
        # Clean a second time doesn't fail.
        # Regression test for https://bugs.launchpad.net/snapcraft/+bug/1497371
        self.copy_project_to_cwd('simple-make')
        self.run_snapcraft('snap')
        self.run_snapcraft('clean')
        self.run_snapcraft('clean')

    # Regression test for LP: #1596596
    def test_clean_invalid_yaml(self):
        self.run_snapcraft('clean', 'invalid-snap')
        self.assertThat('parts', Not(DirExists()))
        self.assertThat('stage', Not(DirExists()))
        self.assertThat('prime', Not(DirExists()))
