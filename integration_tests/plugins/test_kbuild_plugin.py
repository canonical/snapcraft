# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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

from testtools.matchers import FileExists

import integration_tests
import snapcraft
from snapcraft.tests.matchers import HasArchitecture


class KBuildPluginTestCase(integration_tests.TestCase):

    def test_stage(self):
        self.run_snapcraft('stage', 'kbuild-hello')

        binary = os.path.join(self.parts_dir, 'kbuild-hello', 'install', 'bin',
                              'myapp')
        self.assertThat(binary, FileExists())

    def test_cross_compiling(self):
        if snapcraft.ProjectOptions().deb_arch != 'amd64':
            self.skipTest('The test only handles amd64 to arm64')

        self.run_snapcraft(['build', '--target-arch=arm64'], 'kbuild-hello')

        binary = os.path.join(self.parts_dir, 'kbuild-hello', 'install', 'bin',
                              'myapp')
        self.assertThat(binary, HasArchitecture('aarch64'))
