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

import fixtures
import testscenarios

from snapcraft import common
from snapcraft.tests import fixture_setup


class TestCase(testscenarios.WithScenarios, fixtures.TestWithFixtures):

    def setUp(self):
        super().setUp()
        temp_cwd_fixture = fixture_setup.TempCWD()
        self.useFixture(temp_cwd_fixture)
        self.path = temp_cwd_fixture.path
        # Some tests will directly or indirectly change the plugindir, which
        # is a module variable. Make sure that it is returned to the original
        # value when a test ends.
        self.addCleanup(common.set_plugindir, common.get_plugindir())
