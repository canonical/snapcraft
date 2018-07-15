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

from snapcraft.internal.pluginhandler._build_attributes import BuildAttributes

from tests import unit


class BuildAttributesTestCase(unit.TestCase):
    def test_no_system_libraries(self):
        build_attributes = BuildAttributes([])
        self.assertFalse(build_attributes.no_system_libraries())

        build_attributes = BuildAttributes(["no-system-libraries"])
        self.assertTrue(build_attributes.no_system_libraries())
