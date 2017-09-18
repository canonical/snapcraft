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
import subprocess
import tempfile

import yaml
from testtools.matchers import Equals, FileContains, FileExists, Not

import integration_tests
import snapcraft
from snapcraft.tests.matchers import HasArchitecture


class GoPluginTestCase(integration_tests.TestCase):

    def test_stage_go_plugin(self):
        self.run_snapcraft('stage', 'go-hello')

        # XXX go names the binary after the directory. --elopio - 2017-01-25
        binary_output = subprocess.check_output(
            os.path.join(self.stage_dir, 'bin', os.path.basename(self.path)),
            universal_newlines=True)
        self.assertThat(binary_output, Equals('Hello snapcrafter\n'))

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
        if snapcraft.ProjectOptions().deb_arch != 'amd64':
            self.skipTest('The test only handles amd64 to arm64')

        target_arch = 'arm64'
        self.run_snapcraft(['build', '--target-arch={}'.format(target_arch)],
                           'go-hello')
        binary = os.path.join(self.parts_dir, 'go-hello', 'install', 'bin',
                              os.path.basename(self.path))
        self.assertThat(binary, HasArchitecture('aarch64'))

    def test_cross_compiling_with_cgo(self):
        if snapcraft.ProjectOptions().deb_arch != 'amd64':
            self.skipTest('The test only handles amd64 to arm64')

        target_arch = 'arm64'
        self.run_snapcraft(['build', '--target-arch={}'.format(target_arch)],
                           'go-cgo')
        binary = os.path.join(self.parts_dir, 'go-cgo', 'install', 'bin',
                              os.path.basename(self.path))
        self.assertThat(binary, HasArchitecture('aarch64'))

    sources_arm64 = '''
    deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ xenial main restricted
    deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ xenial-updates main restricted
    deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ xenial universe
    deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ xenial-updates universe
    deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ xenial multiverse
    deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports/ xenial-updates multiverse
    deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports xenial-security main restricted
    deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports xenial-security universe
    deb [arch=arm64] http://ports.ubuntu.com/ubuntu-ports xenial-security multiverse'''  # noqa

    def test_cross_compiling_with_cgo_and_target_suffix(self):
        if snapcraft.ProjectOptions().deb_arch != 'amd64':
            self.skipTest('The test only handles amd64 to arm64')

        # Setup the host for cross-compiling to arm64
        subprocess.check_call(['sudo', 'dpkg', '--add-architecture', 'arm64'])
        with tempfile.NamedTemporaryFile() as sources_arch_file:
            sources_arch_file.write(self.sources_arm64.encode())
            sources_arch_file.flush()
            sources_lists = os.path.join('/etc/apt/sources.list.d/',
                                         'ubuntu-{}.list'.format('arm64'))
            subprocess.check_call(['sudo', 'cp',
                                   sources_arch_file.name, sources_lists])
            subprocess.check_call(['sudo', 'chmod', '644', sources_lists])
            loop_and_update = '''
                while sudo fuser /var/lib/apt/lists/lock >/dev/null 2>&1;
                do sleep 1; done;
                sudo apt-get update'''
            subprocess.check_call(['sh', '-c', loop_and_update])

        target_arch = 'arm64'
        self.run_snapcraft(['build', '--target-arch={}'.format(target_arch)],
                           'go-cgo-glib')
        binary = os.path.join(self.parts_dir, 'go-cgo-glib', 'install', 'bin',
                              os.path.basename(self.path))
        self.assertThat(binary, HasArchitecture('aarch64'))
