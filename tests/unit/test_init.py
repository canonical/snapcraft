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
from testtools.matchers import Equals

import snapcraft
from tests import unit


class VersionTestCase(unit.TestCase):
    def test_version_from_snap(self):
        self.useFixture(fixtures.EnvironmentVariable("SNAP_NAME", "snapcraft"))
        self.useFixture(fixtures.EnvironmentVariable("SNAP_VERSION", "3.14"))
        self.assertThat(snapcraft._get_version(), Equals("3.14"))
