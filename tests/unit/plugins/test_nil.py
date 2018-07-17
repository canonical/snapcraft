# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2017-2018 Canonical Ltd
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

from testtools.matchers import Equals

from snapcraft.plugins.nil import NilPlugin
from tests import unit


class TestNilPlugin(unit.TestCase):
    def test_schema(self):
        schema = NilPlugin.schema()
        self.assertThat(
            schema["$schema"], Equals("http://json-schema.org/draft-04/schema#")
        )
        self.assertThat(schema["type"], Equals("object"))
        self.assertFalse(schema["additionalProperties"])
        self.assertFalse(schema["properties"])
