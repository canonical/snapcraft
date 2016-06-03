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
    FileExists,
    Not
)

import integration_tests


class CleanBuildStepBuiltTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()

        self.project_dir = 'independent-parts'
        self.run_snapcraft('build', self.project_dir)
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
        for d in ['builddir', 'bindir']:
            self.assertThat(os.path.join(self.parts['part1'][d], 'file1'),
                            FileExists())
            self.assertThat(os.path.join(self.parts['part2'][d], 'file2'),
                            FileExists())

    def test_clean_build_step(self):
        self.assert_files_exist()

        self.run_snapcraft(['clean', '--step=build'], self.project_dir)

        for part_name, part in self.parts.items():
            self.assertThat(part['builddir'], Not(DirExists()))
            self.assertThat(part['installdir'], Not(DirExists()))
            self.assertThat(part['sourcedir'], DirExists())

        # Now try to build again
        self.run_snapcraft('build', self.project_dir)
        self.assert_files_exist()

    def test_clean_build_step_single_part(self):
        self.assert_files_exist()

        self.run_snapcraft(['clean', 'part1', '--step=build'],
                           self.project_dir)
        self.assertThat(self.parts['part1']['builddir'], Not(DirExists()))
        self.assertThat(self.parts['part1']['installdir'], Not(DirExists()))
        self.assertThat(self.parts['part1']['sourcedir'], DirExists())

        self.assertThat(
            os.path.join(self.parts['part2']['builddir'], 'file2'),
            FileExists())
        self.assertThat(
            os.path.join(self.parts['part2']['bindir'], 'file2'),
            FileExists())

        # Now try to build again
        self.run_snapcraft('build', self.project_dir)
        self.assert_files_exist()


class CleanBuildStepPrimedTestCase(integration_tests.TestCase):

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
        for d in ['builddir', 'bindir']:
            self.assertThat(os.path.join(self.parts['part1'][d], 'file1'),
                            FileExists())
            self.assertThat(os.path.join(self.parts['part2'][d], 'file2'),
                            FileExists())

        self.assertThat(os.path.join(self.snap_bindir, 'file1'), FileExists())
        self.assertThat(os.path.join(self.snap_bindir, 'file2'), FileExists())
        self.assertThat(os.path.join(self.stage_bindir, 'file1'), FileExists())
        self.assertThat(os.path.join(self.stage_bindir, 'file2'), FileExists())

    def test_clean_build_step(self):
        self.assert_files_exist()

        self.run_snapcraft(['clean', '--step=build'], self.project_dir)
        self.assertThat(self.stagedir, Not(DirExists()))
        self.assertThat(self.snapdir, Not(DirExists()))

        for part_name, part in self.parts.items():
            self.assertThat(part['builddir'], Not(DirExists()))
            self.assertThat(part['installdir'], Not(DirExists()))
            self.assertThat(part['sourcedir'], DirExists())

        # Now try to prime again
        self.run_snapcraft('prime', self.project_dir)
        self.assert_files_exist()

    def test_clean_build_step_single_part(self):
        self.assert_files_exist()

        self.run_snapcraft(['clean', 'part1', '--step=build'],
                           self.project_dir)
        self.assertThat(os.path.join(self.stage_bindir, 'file1'),
                        Not(FileExists()))
        self.assertThat(os.path.join(self.stage_bindir, 'file2'), FileExists())
        self.assertThat(os.path.join(self.snap_bindir, 'file1'),
                        Not(FileExists()))
        self.assertThat(os.path.join(self.snap_bindir, 'file2'), FileExists())

        self.assertThat(self.parts['part1']['builddir'], Not(DirExists()))
        self.assertThat(self.parts['part1']['installdir'], Not(DirExists()))
        self.assertThat(self.parts['part1']['sourcedir'], DirExists())

        self.assertThat(
            os.path.join(self.parts['part2']['builddir'], 'file2'),
            FileExists())
        self.assertThat(
            os.path.join(self.parts['part2']['bindir'], 'file2'),
            FileExists())

        # Now try to prime again
        self.run_snapcraft('prime', self.project_dir)
        self.assert_files_exist()
