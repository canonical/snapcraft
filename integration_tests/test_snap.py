# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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
import subprocess

import fixtures
import testtools
from testtools.matchers import (
    EndsWith,
    FileContains,
    FileExists,
    Not,
)

import snapcraft
import integration_tests


class SnapTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()
        self.deb_arch = snapcraft.ProjectOptions().deb_arch

    def test_snap(self):
        project_dir = 'assemble'
        self.run_snapcraft('snap', project_dir)
        os.chdir(project_dir)

        snap_file_path = 'assemble_1.0_{}.snap'.format(self.deb_arch)
        self.assertThat(snap_file_path, FileExists())

        binary1_wrapper_path = os.path.join(
            'prime', 'command-assemble-bin.wrapper')
        with open('binary1.after', 'r') as file_:
            expected_binary1_wrapper = file_.read()
        self.assertThat(
            binary1_wrapper_path, FileContains(expected_binary1_wrapper))

        self.useFixture(
           fixtures.EnvironmentVariable(
                'SNAP', os.path.join(os.getcwd(), 'prime')))
        binary_scenarios = (
            ('command-assemble-service.wrapper', 'service-start\n'),
            ('stop-command-assemble-service.wrapper', 'service-stop\n'),
            ('command-assemble-bin.wrapper', 'binary1\n'),
            ('command-binary2.wrapper', 'binary2\n'),
        )
        for binary, expected_output in binary_scenarios:
            output = subprocess.check_output(
                os.path.join('prime', binary), universal_newlines=True)
            self.assertEqual(expected_output, output)

        with testtools.ExpectedException(subprocess.CalledProcessError):
            subprocess.check_output(
                os.path.join('prime', 'bin', 'not-wrapped'),
                stderr=subprocess.STDOUT)

        self.assertThat(
            os.path.join('prime', 'bin', 'not-wrapped.wrapper'),
            Not(FileExists()))

    def test_snap_default(self):
        project_dir = 'assemble'
        self.run_snapcraft([], project_dir)
        os.chdir(project_dir)

        snap_file_path = 'assemble_1.0_{}.snap'.format(self.deb_arch)
        self.assertThat(snap_file_path, FileExists())

    def test_cleanbuild(self):
        self.skipTest("Fails to run correctly on travis.")
        project_dir = 'assemble'
        self.run_snapcraft('cleanbuild', project_dir)
        os.chdir(project_dir)

        snap_source_path = 'assemble_1.0_source.tar.bz2'
        self.assertThat(snap_source_path, FileExists())

        snap_file_path = 'assemble_1.0_{}.snap'.format(self.deb_arch)
        self.assertThat(snap_file_path, FileExists())

    def test_snap_directory(self):
        project_dir = 'assemble'
        self.run_snapcraft('snap', project_dir)
        os.chdir(project_dir)

        snap_file_path = 'assemble_1.0_{}.snap'.format(self.deb_arch)
        os.remove(snap_file_path)

        # Verify that Snapcraft can snap its own snap directory (this will make
        # sure `snapcraft snap` and `snapcraft snap <directory>` are always in
        # sync).
        self.run_snapcraft(['snap', 'prime'])
        self.assertThat(snap_file_path, FileExists())

    def test_snap_long_output_option(self):
        project_dir = 'assemble'
        self.run_snapcraft(['snap', '--output', 'mysnap.snap'], project_dir)
        os.chdir(project_dir)
        self.assertThat('mysnap.snap', FileExists())

    def test_snap_short_output_option(self):
        project_dir = 'assemble'
        self.run_snapcraft(['snap', '-o', 'mysnap.snap'], project_dir)
        os.chdir(project_dir)
        self.assertThat('mysnap.snap', FileExists())

    def test_error_with_unexistent_build_package(self):
        project_dir = self.copy_project_to_tmp('assemble')
        os.chdir(project_dir)
        with open('snapcraft.yaml', 'a') as yaml_file:
            yaml_file.write('build-packages:\n'
                            '  - inexistent-package\n')

        # We update here to get a clean log/stdout later
        self.run_snapcraft('update', project_dir)

        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, 'snap')
        expected = (
            "Could not find a required package in 'build-packages': "
            '"The cache has no package named \'inexistent-package\'"\n')
        self.assertThat(exception.output, EndsWith(expected))

    def test_snap_with_exposed_files(self):
        project_dir = 'nil-plugin-pkgfilter'
        self.run_snapcraft('stage', project_dir)
        self.assertThat(
            os.path.join(project_dir, 'stage', 'usr', 'bin', 'nmcli'),
            FileExists())

        self.run_snapcraft('snap', project_dir)
        self.assertThat(
            os.path.join(project_dir, 'prime', 'usr', 'bin', 'nmcli'),
            FileExists())
        self.assertThat(
            os.path.join(project_dir, 'prime', 'usr', 'bin', 'nmtui'),
            Not(FileExists()))

    def test_snap_from_snapcraft_init(self):
        self.assertThat('snapcraft.yaml', Not(FileExists()))
        self.run_snapcraft('init')
        self.assertThat('snapcraft.yaml', FileExists())

        self.run_snapcraft('snap')

    def test_error_on_bad_yaml(self):
        project_dir = 'bad-yaml'

        error = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft, 'stage', project_dir)
        self.assertIn(
            "Issues while validating snapcraft.yaml: found character '\\t' "
            "that cannot start any token on line 13 of snapcraft.yaml",
            str(error.output))
