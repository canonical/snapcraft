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
    FileExists
)

import integration_tests


class TarPluginTestCase(integration_tests.TestCase):

    def test_stage_nil_plugin(self):
        self.run_snapcraft('stage', 'tar')

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
                os.path.join(self.stage_dir, expected_file),
                FileExists())
        expected_dirs = [
            'dir-simple',
            'notopdir',
            'destdir1',
            os.path.join('destdir1', 'destdir2')
        ]
        for expected_dir in expected_dirs:
            self.assertThat(
                os.path.join(self.stage_dir, expected_dir),
                DirExists())

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.stage_dir, 'bin', 'test'))
        self.assertThat(binary_output, Equals('tarproject\n'))

        # Regression test for
        # https://bugs.launchpad.net/snapcraft/+bug/1500728
        self.run_snapcraft('pull')
