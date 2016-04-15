# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

from unittest import mock

import snapcraft
from snapcraft import tests


class OptionsTestCase(tests.TestCase):
    @mock.patch('platform.machine', return_value='x86_64')
    def test_amd64_support(self, mock_platform_machine):
        try:
            snapcraft.ProjectOptions()
        except KeyError:
            self.fail('Expected amd64 to be supported')

    @mock.patch('platform.machine', return_value='i686')
    def test_i386_support(self, mock_platform_machine):
        try:
            snapcraft.ProjectOptions()
        except KeyError:
            self.fail('Expected i386 to be supported')

    @mock.patch('platform.machine', return_value='armv7l')
    def test_arm_support(self, mock_platform_machine):
        try:
            snapcraft.ProjectOptions()
        except KeyError:
            self.fail('Expected arm to be supported')

    @mock.patch('platform.machine', return_value='aarch64')
    def test_arm64_support(self, mock_platform_machine):
        try:
            snapcraft.ProjectOptions()
        except KeyError:
            self.fail('Expected arm64 to be supported')

    @mock.patch('platform.machine', return_value='ppc64le')
    def test_ppc_support(self, mock_platform_machine):
        try:
            snapcraft.ProjectOptions()
        except KeyError:
            self.fail('Expected ppc to be supported')

    @mock.patch('platform.machine', return_value='s390x')
    def test_s390x_support(self, mock_platform_machine):
        try:
            snapcraft.ProjectOptions()
        except KeyError:
            self.fail('Expected s390x to be supported')
