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

import os

from testtools.matchers import (
    DirExists,
    Not
)

import integration_tests


class MakePluginTestCase(integration_tests.TestCase):

    def test_stage_make_plugin(self):
        self.run_snapcraft('stage', 'simple-make')

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join('stage', 'bin', 'test'))
        self.assertEqual('Hello world\n', binary_output)

    def test_clean_make_plugin(self):
        self.run_snapcraft('snap', 'simple-make')

        snap_dirs = ('stage', 'parts', 'prime')
        for dir_ in snap_dirs:
            self.assertThat(dir_, DirExists())

        self.run_snapcraft('clean')
        for dir_ in snap_dirs:
            self.assertThat(dir_, Not(DirExists()))

    def test_nonstandard_makefile(self):
        self.run_snapcraft('stage', 'simple-make-nonstandard-makefile')

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join('stage', 'bin', 'test'))
        self.assertEqual('Hello world\n', binary_output)

    def test_stage_make_with_artifacts(self):
        self.run_snapcraft('stage', 'simple-make-artifacts')

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join('stage', 'test'))
        self.assertEqual('Hello World!\n', binary_output)
