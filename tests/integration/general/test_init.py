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

import fixtures
from tests import integration


class InitTestCase(integration.TestCase):
    def test_init_without_locale(self):
        self.useFixture(fixtures.EnvironmentVariable("LC_ALL"))
        self.useFixture(fixtures.EnvironmentVariable("LANG"))

        # This should not throw exceptions
        self.run_snapcraft(["init"])
