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

from testtools import TestCase
from testtools.matchers import Equals

from snapcraft.plugins.v2.nil import NilPlugin


class NilPluginTest(TestCase):
    def test_schema(self):
        schema = NilPlugin.get_schema()

        self.assertThat(
            schema["$schema"], Equals("http://json-schema.org/draft-04/schema#")
        )
        self.assertThat(schema["type"], Equals("object"))
        self.assertFalse(schema["additionalProperties"])
        self.assertFalse(schema["properties"])

    def test_get_build_packages(self):
        plugin = NilPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(plugin.get_build_packages(), Equals(set()))

    def test_get_build_environment(self):
        plugin = NilPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(plugin.get_build_environment(), Equals(dict()))

    def test_get_build_commands(self):
        plugin = NilPlugin(part_name="my-part", options=lambda: None)

        self.assertThat(plugin.get_build_commands(), Equals(list()))
