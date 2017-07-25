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

import snapcraft
import integration_tests
from snapcraft.tests.matchers import HasArchitecture


class KBuildPluginTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()
        self.snaps_dir = os.path.join(os.path.dirname(__file__),
                                      '..', '..', 'demos')

    def test_stage(self):
        self.run_snapcraft('stage', 'busybox')

        binary = os.path.join(self.parts_dir, 'busybox', 'install', 'bin',
                              'busybox')
        self.assertThat(binary, FileExists())

    def test_cross_compiling(self):
        if snapcraft.ProjectOptions().deb_arch != 'amd64':
            self.skipTest('The test only handles amd64 to arm64')

        self.run_snapcraft(['build', '--target-arch=arm64'],
                           'busybox')
        binary = os.path.join(self.parts_dir, 'busybox', 'install', 'bin',
                              'busybox')
        self.assertThat(binary, HasArchitecture('aarch64'))
