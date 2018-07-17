# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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

import fixtures
from snapcraft import ProjectOptions, storeapi, __version__ as snapcraft_version
from tests import unit


class UserAgentTestCase(unit.TestCase):
    def test_user_agent(self):
        arch = ProjectOptions().deb_arch
        expected_pre = "snapcraft/{} ".format(snapcraft_version)
        expected_post = " {} ({})".format(
            "/".join(platform.dist()[0:2]), arch  # i.e. Ubuntu/16.04
        )
        actual = storeapi._agent.get_user_agent()
        self.assertTrue(actual.startswith(expected_pre))
        self.assertTrue(actual.endswith(expected_post))

    def test_in_travis_ci_env(self):
        self.useFixture(fixtures.EnvironmentVariable("TRAVIS_TESTING", "1"))

        self.assertTrue(storeapi._agent._is_ci_env())

    def test_in_autopkgtest_ci_env(self):
        self.useFixture(fixtures.EnvironmentVariable("AUTOPKGTEST_TMP", "1"))

        self.assertTrue(storeapi._agent._is_ci_env())

    def test_not_in_ci_env(self):
        # unset any known testing environment vars
        testing_vars = ["TRAVIS", "AUTHPKGTEST_TMP"]
        vars_to_unset = []
        for env_var in os.environ:
            for test_var in testing_vars:
                if env_var.startswith(test_var):
                    vars_to_unset.append(env_var)

        for var in vars_to_unset:
            self.useFixture(fixtures.EnvironmentVariable(var, None))

        self.assertFalse(storeapi._agent._is_ci_env())
