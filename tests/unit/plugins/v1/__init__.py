# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

from snapcraft.internal.meta.snap import Snap
from snapcraft.project import Project
from tests import unit


class PluginsV1BaseTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.project = Project()
        self.project._snap_meta = Snap(
            name="test-snap", base="core18", confinement="strict"
        )
