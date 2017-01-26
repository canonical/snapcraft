# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016, 2017 Marius Gripsgard (mariogrip@ubuntu.com)
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

from testtools.matchers import FileExists, Not

import integration_tests


class RustPluginTestCase(integration_tests.TestCase):

    def test_stage_rust_plugin(self):
        self.run_snapcraft('stage', 'simple-rust')

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join('stage', 'bin', 'simple-rust'))
        self.assertEqual('There is rust on snaps!\n', binary_output)

    def test_stage_rust_with_revision(self):
        self.run_snapcraft('stage', 'rust-with-revision')

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join('stage', 'bin', 'rust-with-revision'))
        self.assertIn('Rust revision: 1.12.0', binary_output)

    def test_stage_rust_plugin_with_conditional_feature(self):
        self.run_snapcraft('stage', 'rust-with-conditional')

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join('stage', 'bin', 'simple-rust'))
        self.assertEqual('Conditional features work!\n', binary_output)

    def test_stage_rust_with_source_subdir(self):
        self.run_snapcraft('stage', 'rust-subdir')

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join('stage', 'bin', 'rust-subdir'))
        self.assertEqual('Rust in a subdirectory works\n', binary_output)
        # Test for bug https://bugs.launchpad.net/snapcraft/+bug/1654764
        self.assertThat('Cargo.lock', Not(FileExists()))
