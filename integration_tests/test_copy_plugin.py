# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015 Canonical Ltd
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

import testtools
from testtools.matchers import (
    DirExists,
    FileContains,
    FileExists
)
import subprocess

import integration_tests


class CopyPluginTestCase(integration_tests.TestCase):

    def test_stage_copy_plugin_with_files(self):
        project_dir = 'simple-copy'
        self.run_snapcraft('stage', project_dir)
        dest_dir = os.path.join(project_dir, 'stage')

        self.assertThat(
            os.path.join(dest_dir, 'dst'), FileContains('I got copied\n'))
        self.assertThat(
            os.path.join(dest_dir, 'dstdir', 'srcdirfile.txt'),
            FileContains('A file in the source directory\n'))
        self.assertFalse(os.path.exists(os.path.join(dest_dir, 'icon.png')),
            'Icon is not copied')

    def test_stage_copy_plugin_no_destination_no_file(self):
        project_dir = 'simple-copy-nodest-nofile'
        self.run_snapcraft('stage', project_dir)
        dest_dir = os.path.join(project_dir, 'stage')

        self.assertThat(
            os.path.join(dest_dir, 'src'), FileContains('I got copied\n'))
        self.assertThat(
            os.path.join(dest_dir, 'srcdir', 'srcdirfile.txt'),
            FileContains('A file in the source directory\n'))
        self.assertThat(
            os.path.join(dest_dir, 'icon.png'),
            FileExists())

    def test_stage_copy_plugin_with_destination_and_file(self):
        project_dir = 'simple-copy-dest-file'

        with testtools.ExpectedException(subprocess.CalledProcessError):
            self.run_snapcraft('stage', project_dir)

    def test_stage_copy_plugin_with_destination(self):
        project_dir = 'simple-copy-withdest'
        self.run_snapcraft('stage', project_dir)
        dest_dir = os.path.join(project_dir, 'stage', 'subdir')

        self.assertThat(
            os.path.join(dest_dir, 'src'), FileContains('I got copied\n'))
        self.assertThat(
            os.path.join(dest_dir, 'srcdir', 'srcdirfile.txt'),
            FileContains('A file in the source directory\n'))
        self.assertThat(
            os.path.join(dest_dir, 'icon.png'),
            FileExists())

    def test_stage_copy_plugin_with_source(self):
        project_dir = 'simple-copy-with-source'
        self.run_snapcraft('stage', project_dir)
        dest_dir = os.path.join(project_dir, 'stage')

        self.assertThat(
            os.path.join(dest_dir, 'dst'), FileContains('I got copied\n'))
        self.assertThat(
            os.path.join(dest_dir, 'dstdir', 'srcdirfile.txt'),
            FileContains('A file in the source directory\n'))
        self.assertFalse(os.path.exists(os.path.join(dest_dir, 'icon.png')),
            'Icon is not copied')

    def test_stage_copy_plugin_tar(self):
        project_dir = 'simple-copy-tar'
        self.run_snapcraft('stage', project_dir)

        expected_files = [
            'flat',
            os.path.join('flatdir', 'flat2'),
            'onedeep',
            os.path.join('onedeepdir', 'onedeep2'),
            'oneflat',
            'top-simple',
            'notop',
            'parent',
            'slash',
            'readonly_file',
            os.path.join('destdir1', 'destdir2', 'top-simple')
        ]
        for expected_file in expected_files:
            self.assertThat(
                os.path.join(project_dir, 'stage', expected_file),
                FileExists())
        expected_dirs = [
            'dir-simple',
            'notopdir',
            'destdir1',
            os.path.join('destdir1', 'destdir2')
        ]
        for expected_dir in expected_dirs:
            self.assertThat(
                os.path.join(project_dir, 'stage', expected_dir),
                DirExists())

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join('stage', 'bin', 'test'), cwd=project_dir)
        self.assertEqual('tarproject\n', binary_output)

        # Regression test for
        # https://bugs.launchpad.net/snapcraft/+bug/1500728
        self.run_snapcraft('pull', project_dir)

    def test_snap_copy_plugin_empty(self):
        project_dir = 'simple-copy-empty'
        self.run_snapcraft('snap', project_dir)

        dirs = os.listdir(os.path.join(project_dir, 'snap'))
        self.assertEqual(['meta'], dirs)
