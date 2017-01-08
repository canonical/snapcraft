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

import integration_tests
import snapcraft


class RustPluginTestCase(integration_tests.TestCase):

    def test_stage_rust_plugin(self):
        project_dir = 'simple-rust'
        self.run_snapcraft('stage', project_dir)

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join('stage', 'bin', 'simple-rust'), cwd=project_dir)
        self.assertEqual('There is rust on snaps!\n', binary_output)

    def test_stage_rust_with_revision(self):
        if snapcraft.ProjectOptions().deb_arch == 'armhf':
            self.skipTest('The rust revision chosen fails on armhf')

        project_dir = 'rust-with-revision'
        self.run_snapcraft('stage', project_dir)

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join('stage', 'bin', 'rust-with-revision'),
            cwd=project_dir)
        self.assertIn('Rust revision: 1.13.0', binary_output)

    def test_stage_rust_plugin_with_conditional_feature(self):
        project_dir = 'rust-with-conditional'
        self.run_snapcraft('stage', project_dir)

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join('stage', 'bin', 'simple-rust'), cwd=project_dir)
        self.assertEqual('Conditional features work!\n', binary_output)
