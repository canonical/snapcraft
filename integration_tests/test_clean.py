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

import subprocess

from testtools.matchers import (
    Contains,
    DirExists,
    Equals,
    Not
)

import integration_tests


class CleanTestCase(integration_tests.TestCase):

    def test_clean(self):
        self.copy_project_to_cwd('make-hello')
        self.run_snapcraft('snap')

        snap_dirs = (self.parts_dir, self.stage_dir, self.prime_dir)
        for dir_ in snap_dirs:
            self.assertThat(dir_, DirExists())

        self.run_snapcraft('clean')
        for dir_ in snap_dirs:
            self.assertThat(dir_, Not(DirExists()))

    def _clean_invalid_part(self, debug):
        self.copy_project_to_cwd('make-hello')
        self.run_snapcraft('snap')

        raised = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft,
            ['clean', 'invalid-part'], debug=debug)
        self.assertThat(raised.returncode, Equals(2))
        self.assertThat(
            raised.output,
            Contains("The part named 'invalid-part' is not defined"))
        return raised.output

    def test_clean_invalid_part_no_traceback_without_debug(self):
        self.assertThat(
            self._clean_invalid_part(False), Not(Contains("Traceback")))

    def test_clean_invalid_part_traceback_with_debug(self):
        self.assertThat(
            self._clean_invalid_part(True), Contains("Traceback"))

    def test_clean_again(self):
        # Clean a second time doesn't fail.
        # Regression test for https://bugs.launchpad.net/snapcraft/+bug/1497371
        self.copy_project_to_cwd('make-hello')
        self.run_snapcraft('snap')
        self.run_snapcraft('clean')
        self.run_snapcraft('clean')

    # Regression test for LP: #1596596
    def test_clean_invalid_yaml(self):
        self.run_snapcraft('clean', 'invalid-snap')
        self.assertThat(self.parts_dir, Not(DirExists()))
        self.assertThat(self.stage_dir, Not(DirExists()))
        self.assertThat(self.prime_dir, Not(DirExists()))
