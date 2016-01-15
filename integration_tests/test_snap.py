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
import subprocess

import fixtures
import testtools
from testtools.matchers import (
    FileContains,
    FileExists,
    Not,
)

import integration_tests


def _get_deb_arch():
    return _run_dpkg_architecture('-qDEB_BUILD_ARCH')


def _get_deb_multiarch():
    return _run_dpkg_architecture('-qDEB_BUILD_MULTIARCH')


def _run_dpkg_architecture(arg):
    return subprocess.check_output(
        ['dpkg-architecture', arg], universal_newlines=True).strip()


class SnapTestCase(integration_tests.TestCase):

    def test_snap(self):
        project_dir = 'assemble'
        self.run_snapcraft('snap', project_dir)
        os.chdir(project_dir)

        snap_file_path = 'assemble_1.0_{}.snap'.format(_get_deb_arch())
        self.assertThat(snap_file_path, FileExists())

        binary1_wrapper_path = os.path.join(
            'snap', 'command-assemble-bin.wrapper')
        with open('binary1.after', 'r') as file_:
            binary1_after = file_.read()
        expected_binary1_wrapper = binary1_after.replace(
            '@MULTIARCH@', _get_deb_multiarch())
        self.assertThat(
            binary1_wrapper_path, FileContains(expected_binary1_wrapper))

        self.useFixture(
           fixtures.EnvironmentVariable(
                'SNAP', os.path.join(os.getcwd(), 'snap')))
        binary_scenarios = (
            ('command-assemble-service.wrapper', 'service-start\n'),
            ('stop-command-assemble-service.wrapper', 'service-stop\n'),
            ('command-assemble-bin.wrapper', 'binary1\n'),
            ('command-binary2.wrapper', 'binary2\n'),
        )
        for binary, expected_output in binary_scenarios:
            output = subprocess.check_output(
                os.path.join('snap', binary), universal_newlines=True)
            self.assertEqual(expected_output, output)

        with testtools.ExpectedException(subprocess.CalledProcessError):
            subprocess.check_output(
                os.path.join('snap', 'bin', 'not-wrapped'),
                stderr=subprocess.STDOUT)

        self.assertThat(
            os.path.join('snap', 'bin', 'not-wrapped.wrapper'),
            Not(FileExists()))

    def test_error_with_unexistent_build_package(self):
        project_dir = self.copy_project_to_tmp('assemble')
        os.chdir(project_dir)
        with open('snapcraft.yaml', 'a') as yaml_file:
            yaml_file.write('build-packages:\n'
                            '  - inexistent-package\n')

        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, 'snap')
        expected = (
            'Could not find all the "build-packages" required in '
            'snapcraft.yaml\n')
        self.assertEqual(expected, exception.output)

    def test_snap_with_exposed_files(self):
        project_dir = 'nil-plugin-pkgfilter'
        self.run_snapcraft('stage', project_dir)
        self.assertThat(
            os.path.join(project_dir, 'stage', 'usr', 'bin', 'nmcli'),
            FileExists())

        self.run_snapcraft('snap', project_dir)
        self.assertThat(
            os.path.join(project_dir, 'snap', 'usr', 'bin', 'nmcli'),
            FileExists())
        self.assertThat(
            os.path.join(project_dir, 'snap', 'usr', 'bin', 'nmtui'),
            Not(FileExists()))
