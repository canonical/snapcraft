# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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

from testtools.matchers import Contains, DirExists, FileExists, Not

from tests import integration


class CleanStageStepStagedTestCase(integration.TestCase):
    def setUp(self):
        super().setUp()

        self.copy_project_to_cwd("independent-parts")
        self.run_snapcraft("stage")
        self.bindir = os.path.join(self.stage_dir, "bin")

    def assert_files_exist(self):
        self.assertThat(os.path.join(self.bindir, "file1"), FileExists())
        self.assertThat(os.path.join(self.bindir, "file2"), FileExists())

    def test_clean_stage_step(self):
        self.assert_files_exist()

        output = self.run_snapcraft(["clean", "--step=stage"], debug=False)
        self.assertThat(self.stage_dir, Not(DirExists()))
        self.assertThat(self.parts_dir, DirExists())

        # Assert that the priming and staging areas were removed wholesale, not
        # a part at a time (since we didn't specify any parts).
        self.assertThat(output, Contains("Cleaning up priming area"))
        self.assertThat(output, Contains("Cleaning up staging area"))
        self.expectThat(output, Not(Contains("part1")))
        self.expectThat(output, Not(Contains("part2")))

        # Now try to stage again
        self.run_snapcraft("stage")
        self.assert_files_exist()

    def test_clean_stage_step_single_part(self):
        self.assert_files_exist()

        self.run_snapcraft(["clean", "part1", "--step=stage"])
        self.assertThat(os.path.join(self.bindir, "file1"), Not(FileExists()))
        self.assertThat(os.path.join(self.bindir, "file2"), FileExists())
        self.assertThat(self.parts_dir, DirExists())

        # Now try to stage again
        self.run_snapcraft("stage")
        self.assert_files_exist()


class CleanStageStepPrimedTestCase(integration.TestCase):
    def setUp(self):
        super().setUp()

        self.copy_project_to_cwd("independent-parts")
        self.run_snapcraft("prime")

        self.snap_bindir = os.path.join(self.prime_dir, "bin")
        self.stage_bindir = os.path.join(self.stage_dir, "bin")

    def assert_files_exist(self):
        self.assertThat(os.path.join(self.snap_bindir, "file1"), FileExists())
        self.assertThat(os.path.join(self.snap_bindir, "file2"), FileExists())
        self.assertThat(os.path.join(self.stage_bindir, "file1"), FileExists())
        self.assertThat(os.path.join(self.stage_bindir, "file2"), FileExists())

    def test_clean_stage_step(self):
        self.assert_files_exist()

        self.run_snapcraft(["clean", "--step=stage"])
        self.assertThat(self.stage_dir, Not(DirExists()))
        self.assertThat(self.prime_dir, Not(DirExists()))
        self.assertThat(self.parts_dir, DirExists())

        # Now try to prime again
        self.run_snapcraft("prime")
        self.assert_files_exist()

    def test_clean_stage_step_single_part(self):
        self.assert_files_exist()

        self.run_snapcraft(["clean", "part1", "--step=stage"])
        self.assertThat(os.path.join(self.stage_bindir, "file1"), Not(FileExists()))
        self.assertThat(os.path.join(self.stage_bindir, "file2"), FileExists())
        self.assertThat(os.path.join(self.snap_bindir, "file1"), Not(FileExists()))
        self.assertThat(os.path.join(self.snap_bindir, "file2"), FileExists())
        self.assertThat(self.parts_dir, DirExists())

        # Now try to prime again
        self.run_snapcraft("prime")
        self.assert_files_exist()
