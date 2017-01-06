# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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
    Contains,
    DirExists,
    FileExists,
    Not
)

import integration_tests


class CleanPrimeStepTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()

        self.project_dir = 'independent-parts'
        self.run_snapcraft('prime', self.project_dir)

    def test_clean_prime_step(self):
        snapdir = os.path.join(self.project_dir, 'prime')
        bindir = os.path.join(snapdir, 'bin')
        self.assertThat(os.path.join(bindir, 'file1'), FileExists())
        self.assertThat(os.path.join(bindir, 'file2'), FileExists())

        output = self.run_snapcraft(
            ['clean', '--step=prime'], self.project_dir, debug=False)
        self.assertThat(snapdir, Not(DirExists()))
        self.assertThat(os.path.join(self.project_dir, 'stage'), DirExists())
        self.assertThat(os.path.join(self.project_dir, 'parts'), DirExists())

        # Assert that the priming area was removed wholesale, not a part at a
        # time (since we didn't specify any parts).
        self.assertThat(output, Contains("Cleaning up priming area"))
        self.expectThat(output, Not(Contains('part1')))
        self.expectThat(output, Not(Contains('part2')))

        # Now try to prime again
        self.run_snapcraft('prime', self.project_dir)
        self.assertThat(os.path.join(bindir, 'file1'), FileExists())
        self.assertThat(os.path.join(bindir, 'file2'), FileExists())

    def test_clean_prime_step_single_part(self):
        bindir = os.path.join(self.project_dir, 'prime', 'bin')
        self.assertThat(os.path.join(bindir, 'file1'), FileExists())
        self.assertThat(os.path.join(bindir, 'file2'), FileExists())

        self.run_snapcraft(['clean', 'part1', '--step=prime'],
                           self.project_dir)
        self.assertThat(os.path.join(bindir, 'file1'), Not(FileExists()))
        self.assertThat(os.path.join(bindir, 'file2'), FileExists())
        self.assertThat(os.path.join(self.project_dir, 'stage'), DirExists())
        self.assertThat(os.path.join(self.project_dir, 'parts'), DirExists())

        # Now try to prime again
        self.run_snapcraft('prime', self.project_dir)
        self.assertThat(os.path.join(bindir, 'file1'), FileExists())
        self.assertThat(os.path.join(bindir, 'file2'), FileExists())

    def test_clean_with_deprecated_strip_step(self):
        snapdir = os.path.join(self.project_dir, 'prime')
        bindir = os.path.join(snapdir, 'bin')
        self.assertThat(os.path.join(bindir, 'file1'), FileExists())
        self.assertThat(os.path.join(bindir, 'file2'), FileExists())

        self.run_snapcraft(['clean', '--step=strip'], self.project_dir)
        self.assertThat(snapdir, Not(DirExists()))
        self.assertThat(os.path.join(self.project_dir, 'stage'), DirExists())
        self.assertThat(os.path.join(self.project_dir, 'parts'), DirExists())

        # Now try to prime again
        self.run_snapcraft('prime', self.project_dir)
        self.assertThat(os.path.join(bindir, 'file1'), FileExists())
        self.assertThat(os.path.join(bindir, 'file2'), FileExists())
