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


class CleanPrimeStepTestCase(integration.TestCase):
    def setUp(self):
        super().setUp()

        self.copy_project_to_cwd("independent-parts")
        self.run_snapcraft("prime")

    def test_clean_prime_step(self):
        bindir = os.path.join(self.prime_dir, "bin")
        self.assertThat(os.path.join(bindir, "file1"), FileExists())
        self.assertThat(os.path.join(bindir, "file2"), FileExists())

        output = self.run_snapcraft(["clean", "--step=prime"], debug=False)
        self.assertThat(self.prime_dir, Not(DirExists()))
        self.assertThat(self.stage_dir, DirExists())
        self.assertThat(self.parts_dir, DirExists())

        # Assert that the priming area was removed wholesale, not a part at a
        # time (since we didn't specify any parts).
        self.assertThat(output, Contains("Cleaning up priming area"))
        self.expectThat(output, Not(Contains("part1")))
        self.expectThat(output, Not(Contains("part2")))

        # Now try to prime again
        self.run_snapcraft("prime")
        self.assertThat(os.path.join(bindir, "file1"), FileExists())
        self.assertThat(os.path.join(bindir, "file2"), FileExists())

    def test_clean_prime_step_single_part(self):
        bindir = os.path.join(self.prime_dir, "bin")
        self.assertThat(os.path.join(bindir, "file1"), FileExists())
        self.assertThat(os.path.join(bindir, "file2"), FileExists())

        self.run_snapcraft(["clean", "part1", "--step=prime"])
        self.assertThat(os.path.join(bindir, "file1"), Not(FileExists()))
        self.assertThat(os.path.join(bindir, "file2"), FileExists())
        self.assertThat(self.stage_dir, DirExists())
        self.assertThat(self.parts_dir, DirExists())

        # Now try to prime again
        self.run_snapcraft("prime")
        self.assertThat(os.path.join(bindir, "file1"), FileExists())
        self.assertThat(os.path.join(bindir, "file2"), FileExists())

    def test_clean_with_deprecated_strip_step(self):
        bindir = os.path.join(self.prime_dir, "bin")
        self.assertThat(os.path.join(bindir, "file1"), FileExists())
        self.assertThat(os.path.join(bindir, "file2"), FileExists())

        self.run_snapcraft(["clean", "--step=strip"])
        self.assertThat(self.prime_dir, Not(DirExists()))
        self.assertThat(self.stage_dir, DirExists())
        self.assertThat(self.parts_dir, DirExists())

        # Now try to prime again
        self.run_snapcraft("prime")
        self.assertThat(os.path.join(bindir, "file1"), FileExists())
        self.assertThat(os.path.join(bindir, "file2"), FileExists())
