# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Marius Gripsgard (mariogrip@ubuntu.com)
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
import re
import subprocess

import fixtures
import testscenarios
import yaml
from testtools.matchers import FileExists, MatchesRegex, Not

import snapcraft
import integration_tests


class RustPluginBaseTestCase(integration_tests.TestCase):

    def run_snapcraft(self, command, project_dir=None, debug=True):
        try:
            failed = True
            super().run_snapcraft(command, project_dir, debug)
            failed = False
        except subprocess.CalledProcessError:
            if snapcraft.ProjectOptions().deb_arch == 'arm64':
                # https://github.com/rust-lang/rustup.sh/issues/82
                self.expectFailure(
                    'The rustup script does not support arm64.',
                    self.assertFalse, failed)
            else:
                raise


class RustPluginTestCase(RustPluginBaseTestCase):

    def test_stage_rust_plugin(self):
        self.run_snapcraft('stage', 'rust-hello')

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.stage_dir, 'bin', 'rust-hello'))
        self.assertEqual('There is rust on snaps!\n', binary_output)

    def test_stage_rust_with_revision(self):
        self.run_snapcraft('stage', 'rust-with-revision')

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.stage_dir, 'bin', 'rust-with-revision'))
        self.assertIn('Rust revision: 1.12.0', binary_output)

    def test_stage_rust_plugin_with_conditional_feature(self):
        self.run_snapcraft('stage', 'rust-with-conditional')

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.stage_dir, 'bin', 'simple-rust'))
        self.assertEqual('Conditional features work!\n', binary_output)

    def test_stage_rust_with_source_subdir(self):
        self.run_snapcraft('stage', 'rust-subdir')

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.stage_dir, 'bin', 'rust-subdir'))
        self.assertEqual('Rust in a subdirectory works\n', binary_output)
        # Test for bug https://bugs.launchpad.net/snapcraft/+bug/1654764
        self.assertThat('Cargo.lock', Not(FileExists()))


class RustPluginConfinementTestCase(testscenarios.WithScenarios,
                                    RustPluginBaseTestCase):

    scenarios = (
        ('classic', dict(confinement='classic',
                         startswith='/snap/core/current/lib')),
        ('strict', dict(confinement='strict',
                        startswith='/lib')),
    )

    def _set_confinement(self, snapcraft_yaml_file):
        with open(snapcraft_yaml_file) as f:
            snapcraft_yaml = yaml.load(f)
        snapcraft_yaml['confinement'] = self.confinement
        with open(snapcraft_yaml_file, 'w') as f:
            yaml.dump(snapcraft_yaml, f)

    def test_build(self):
        self.useFixture(fixtures.EnvironmentVariable(
                'SNAPCRAFT_SETUP_CORE', '1'))
        self.copy_project_to_cwd('rust-hello')
        self._set_confinement('snapcraft.yaml')

        self.run_snapcraft('build')

        binary = os.path.join(self.parts_dir, 'rust-hello', 'install',
                              'bin', 'rust-hello')
        output = subprocess.check_output(['readelf', '--program', binary])
        output = output.decode('utf-8')

        expected = '.*Requesting program interpreter: {}.*'.format(
            self.startswith)
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))
