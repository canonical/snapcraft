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

import os
import platform

from snapcraft import (
    ProjectOptions,
    storeapi,
    tests,
    __version__ as snapcraft_version,
)


class UserAgentTestCase(tests.TestCase):

    def test_user_agent(self):
        arch = ProjectOptions().deb_arch
        expected_pre = 'snapcraft/{} '.format(snapcraft_version)
        expected_post = ' {} ({})'.format(
            '/'.join(platform.dist()[0:2]),  # i.e. Ubuntu/16.04
            arch,
        )
        actual = storeapi._agent._get_user_agent()
        self.assertTrue(actual.startswith(expected_pre))
        self.assertTrue(actual.endswith(expected_post))

    def test_in_ci_env(self):
        os.environ['TRAVIS_STUFF'] = 'stuff'

        self.assertTrue(storeapi._agent._is_ci_env())

    def test_not_in_ci_env(self):
        os.environ = {}

        self.assertFalse(storeapi._agent._is_ci_env())
