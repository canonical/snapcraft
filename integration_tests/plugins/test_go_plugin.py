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
        self.assertEqual('Hello snapcrafter\n', binary_output)

    def test_building_multiple_main_packages(self):
        self.run_snapcraft('stage', 'go-with-multiple-main-packages')

    def test_cross_compiling(self):
        if snapcraft.ProjectOptions().deb_arch != 'amd64':
            self.skipTest('The test only handles amd64 to armhf')

        target_arch = 'arm64'
        self.run_snapcraft(['build', '--target-arch={}'.format(target_arch)],
                           'go-hello')
        binary = os.path.join(self.parts_dir, 'go-hello', 'install', 'bin',
                              os.path.basename(self.path))
        self.assertThat(binary, HasArchitecture('aarch64'))

    def test_cross_compiling_with_cgo(self):
        if snapcraft.ProjectOptions().deb_arch != 'amd64':
            self.skipTest('The test only handles amd64 to armhf')

        target_arch = 'arm64'
        self.run_snapcraft(['build', '--target-arch={}'.format(target_arch)],
                           'go-cgo')
        binary = os.path.join(self.parts_dir, 'go-cgo', 'install', 'bin',
                              os.path.basename(self.path))
        self.assertThat(binary, HasArchitecture('aarch64'))
