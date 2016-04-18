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

from snapcraft.plugins.nil import NilPlugin
from snapcraft.tests import TestCase


class TestNilPlugin(TestCase):
    def test_schema(self):
        schema = NilPlugin.schema()
        self.assertEqual('http://json-schema.org/draft-04/schema#',
                         schema['$schema'])
        self.assertEqual('object', schema['type'])
        self.assertFalse(schema['additionalProperties'])
        self.assertFalse(schema['properties'])
