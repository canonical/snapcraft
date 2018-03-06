# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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

from testtools.matchers import Equals

from tests import integration
from tests.matchers import HasArchitecture


class AutotoolsPluginTestCase(integration.TestCase):

    def test_stage(self):
        self.run_snapcraft('stage', 'autotools-hello')

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.stage_dir, 'bin', 'hello'))
        self.assertThat(binary_output, Equals('Hello, world!\n'))

    def test_cross_compiling(self):
        if self.deb_arch != 'amd64':
            self.skipTest('The test only handles amd64 to arm64')

        self.run_snapcraft(['build', '--target-arch=arm64'],
                           'autotools-hello')
        binary = os.path.join(self.parts_dir, 'make-project', 'install', 'bin',
                              'hello')
        self.assertThat(binary, HasArchitecture('aarch64'))
