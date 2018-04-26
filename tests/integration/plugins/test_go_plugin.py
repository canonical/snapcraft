# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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

import yaml
from testtools.matchers import (Equals, FileContains, FileExists, MatchesRegex,
                                Not)

from tests import integration, os_release
from tests.matchers import HasArchitecture


class GoPluginTestCase(integration.TestCase):

    def test_stage_go_plugin(self):
        self.run_snapcraft('stage', 'go-hello')

        # XXX go names the binary after the directory. --elopio - 2017-01-25
        binary_output = subprocess.check_output(
            os.path.join(self.stage_dir, 'bin', os.path.basename(self.path)),
            universal_newlines=True)
        self.assertThat(binary_output, Equals('Hello snapcrafter\n'))

    def test_classic_with_conflicting_build_id(self):
        # TODO find a faster test to verify LP: #1736861
        if os.environ.get('ADT_TEST') and self.deb_arch == 'armhf':
            self.skipTest("The autopkgtest armhf runners can't install snaps")

        self.copy_project_to_cwd('go-gotty')
        if os_release.get_version_codename() != 'xenial':
            snapcraft_yaml_file = 'snapcraft.yaml'
            with open(snapcraft_yaml_file) as f:
                snapcraft_yaml = yaml.load(f)
                snapcraft_yaml['parts']['gotty']['stage-packages'] = ['libc6']
            with open(snapcraft_yaml_file, 'w') as f:
                yaml.dump(snapcraft_yaml, f)

        self.run_snapcraft('prime')

        bin_path = os.path.join(self.prime_dir, 'bin', 'gotty')

        self.assertThat(bin_path, FileExists())

        interpreter = subprocess.check_output([
            self.patchelf_command, '--print-interpreter', bin_path]).decode()
        # On anything greater than xenial we will have a libc6 discrepancy
        if os_release.get_version_codename() == 'xenial':
            expected_interpreter = r'^/snap/core/current/.*'
        else:
            expected_interpreter = r'^/snap/gotty/current/.*'
        self.assertThat(interpreter, MatchesRegex(expected_interpreter))

    def test_building_multiple_main_packages(self):
        self.run_snapcraft('stage', 'go-with-multiple-main-packages')

        for bin in ['main1', 'main2', 'main3']:
            self.assertThat(os.path.join('stage', 'bin', bin), FileExists())

    def test_building_multiple_main_packages_without_go_packages(self):
        self.copy_project_to_cwd('go-with-multiple-main-packages')

        snapcraft_yaml_file = 'snapcraft.yaml'
        with open(snapcraft_yaml_file) as f:
            snapcraft_yaml = yaml.load(f)
        del snapcraft_yaml['parts']['multiple-mains']['go-packages']
        with open(snapcraft_yaml_file, 'w') as f:
            yaml.dump(snapcraft_yaml, f)

        self.assertThat(snapcraft_yaml_file, Not(FileContains('go-packages')))
        self.run_snapcraft('stage')

        for bin in ['main1', 'main2', 'main3']:
            self.assertThat(os.path.join('stage', 'bin', bin), FileExists())

    def test_cross_compiling(self):
        if self.deb_arch != 'amd64':
            self.skipTest('The test only handles amd64 to arm64')

        target_arch = 'arm64'
        self.run_snapcraft(['build', '--target-arch={}'.format(target_arch)],
                           'go-hello')
        binary = os.path.join(self.parts_dir, 'go-hello', 'install', 'bin',
                              os.path.basename(self.path))
        self.assertThat(binary, HasArchitecture('aarch64'))

    def test_cross_compiling_with_cgo(self):
        if self.deb_arch != 'amd64':
            self.skipTest('The test only handles amd64 to arm64')

        target_arch = 'arm64'
        self.run_snapcraft(['build', '--target-arch={}'.format(target_arch)],
                           'go-cgo')
        binary = os.path.join(self.parts_dir, 'go-cgo', 'install', 'bin',
                              os.path.basename(self.path))
        self.assertThat(binary, HasArchitecture('aarch64'))
