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
    DirExists,
    Equals,
    FileExists,
    Not
)

import integration_tests


class CleanPullStepPulledTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()

        self.project_dir = 'independent-parts'
        self.run_snapcraft('pull', self.project_dir)
        self.partsdir = os.path.join(self.project_dir, 'parts')
        self.part1_sourcedir = os.path.join(self.partsdir, 'part1', 'src')
        self.part2_sourcedir = os.path.join(self.partsdir, 'part2', 'src')

    def assert_files_exist(self):
        self.assertThat(os.path.join(self.part1_sourcedir, 'file1'),
                        FileExists())
        self.assertThat(os.path.join(self.part2_sourcedir, 'file2'),
                        FileExists())

    def test_clean_pull_step(self):
        self.assert_files_exist()

        self.run_snapcraft(['clean', '--step=pull'], self.project_dir)
        self.assertThat(self.part1_sourcedir, Not(DirExists()))
        self.assertThat(self.part2_sourcedir, Not(DirExists()))

        # Now try to pull again
        self.run_snapcraft('pull', self.project_dir)
        self.assert_files_exist()

    def test_clean_pull_step_single_part(self):
        self.assert_files_exist()

        self.run_snapcraft(['clean', 'part1', '--step=pull'],
                           self.project_dir)
        self.assertThat(self.part1_sourcedir, Not(DirExists()))
        self.assertThat(os.path.join(self.part2_sourcedir, 'file2'),
                        FileExists())

        # Now try to pull again
        self.run_snapcraft('pull', self.project_dir)
        self.assert_files_exist()


class CleanPullStepPrimedTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()

        self.project_dir = 'independent-parts'
        self.run_snapcraft('prime', self.project_dir)

        self.snapdir = os.path.join(self.project_dir, 'prime')
        self.snap_bindir = os.path.join(self.snapdir, 'bin')
        self.stagedir = os.path.join(self.project_dir, 'stage')
        self.stage_bindir = os.path.join(self.stagedir, 'bin')
        self.partsdir = os.path.join(self.project_dir, 'parts')
        self.parts = {}
        for part in ['part1', 'part2']:
            partdir = os.path.join(self.partsdir, part)
            self.parts[part] = {
                'partdir': partdir,
                'sourcedir': os.path.join(partdir, 'src'),
                'builddir': os.path.join(partdir, 'build'),
                'installdir': os.path.join(partdir, 'install'),
                'bindir': os.path.join(partdir, 'install', 'bin'),
            }

    def assert_files_exist(self):
        for d in ['builddir', 'bindir', 'sourcedir']:
            self.assertThat(os.path.join(self.parts['part1'][d], 'file1'),
                            FileExists())
            self.assertThat(os.path.join(self.parts['part2'][d], 'file2'),
                            FileExists())

        self.assertThat(os.path.join(self.snap_bindir, 'file1'), FileExists())
        self.assertThat(os.path.join(self.snap_bindir, 'file2'), FileExists())
        self.assertThat(os.path.join(self.stage_bindir, 'file1'), FileExists())
        self.assertThat(os.path.join(self.stage_bindir, 'file2'), FileExists())

    def test_clean_pull_step(self):
        self.assert_files_exist()

        output = self.run_snapcraft(['clean', '--step=pull'], self.project_dir,
                                    debug=False)
        self.assertThat(self.stagedir, Not(DirExists()))
        self.assertThat(self.snapdir, Not(DirExists()))

        for part_name, part in self.parts.items():
            self.assertThat(part['builddir'], Not(DirExists()))
            self.assertThat(part['installdir'], Not(DirExists()))
            self.assertThat(part['sourcedir'], Not(DirExists()))

        # Assert that the priming and staging areas were removed wholesale, not
        # a part at a time (since we didn't specify any parts).
        output = output.strip().split('\n')
        self.expectThat(output, Equals([
            'Cleaning up priming area',
            'Cleaning up staging area',
            'Cleaning up parts directory'
        ]))

        # Now try to prime again
        self.run_snapcraft('prime', self.project_dir)
        self.assert_files_exist()

    def test_clean_pull_step_single_part(self):
        self.assert_files_exist()

        self.run_snapcraft(['clean', 'part1', '--step=pull'],
                           self.project_dir)
        self.assertThat(os.path.join(self.stage_bindir, 'file1'),
                        Not(FileExists()))
        self.assertThat(os.path.join(self.stage_bindir, 'file2'), FileExists())
        self.assertThat(os.path.join(self.snap_bindir, 'file1'),
                        Not(FileExists()))
        self.assertThat(os.path.join(self.snap_bindir, 'file2'), FileExists())

        self.assertThat(self.parts['part1']['builddir'], Not(DirExists()))
        self.assertThat(self.parts['part1']['installdir'], Not(DirExists()))
        self.assertThat(self.parts['part1']['sourcedir'], Not(DirExists()))

        self.assertThat(
            os.path.join(self.parts['part2']['builddir'], 'file2'),
            FileExists())
        self.assertThat(
            os.path.join(self.parts['part2']['bindir'], 'file2'),
            FileExists())
        self.assertThat(
            os.path.join(self.parts['part2']['sourcedir'], 'file2'),
            FileExists())

        # Now try to prime again
        self.run_snapcraft('prime', self.project_dir)
        self.assert_files_exist()
