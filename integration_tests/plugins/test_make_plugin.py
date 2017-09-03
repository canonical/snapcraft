# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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
    Equals,
    Not
)

import integration_tests


class MakePluginTestCase(integration_tests.TestCase):

    def test_stage_make_plugin(self):
        self.run_snapcraft('stage', 'make-hello')

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.stage_dir, 'bin', 'test'))
        self.assertThat(binary_output, Equals('Hello world\n'))

    def test_clean_make_plugin(self):
        self.run_snapcraft('snap', 'make-hello')

        snap_dirs = (self.parts_dir, self.stage_dir, self.prime_dir)
        for dir_ in snap_dirs:
            self.assertThat(dir_, DirExists())

        self.run_snapcraft('clean')
        for dir_ in snap_dirs:
            self.assertThat(dir_, Not(DirExists()))

    def test_nonstandard_makefile(self):
        self.run_snapcraft('stage', 'make-with-nonstandard-makefile')

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.stage_dir, 'bin', 'test'))
        self.assertThat(binary_output, Equals('Hello world\n'))

    def test_stage_make_with_artifacts(self):
        self.run_snapcraft('stage', 'make-with-artifacts')

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.stage_dir, 'test'))
        self.assertThat(binary_output, Equals('Hello World!\n'))
