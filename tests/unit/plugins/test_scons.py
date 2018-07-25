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

import os

from unittest import mock
from testtools.matchers import Equals, HasLength

import snapcraft
from snapcraft.plugins import scons
from tests import unit


class SconsPluginTestCase(unit.TestCase):
    """Plugin to provide snapcraft support for the scons build system"""

    def setUp(self):
        super(SconsPluginTestCase, self).setUp()

        class Options:
            """Internal Options Class matching the Scons plugin"""

            scons_options = ["--debug=explain"]

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

    def test_schema(self):
        """Test validity of the Scons Plugin schema"""
        schema = scons.SconsPlugin.schema()

        # Verify the presence of all properties
        properties = schema["properties"]
        self.assertTrue(
            "scons-options" in properties,
            'Expected "scons-options" to be included in ' "properties",
        )

        scons_options = properties["scons-options"]

        self.assertTrue(
            "type" in scons_options, 'Expected "type" to be included in "scons-options"'
        )
        self.assertThat(
            scons_options["type"],
            Equals("array"),
            'Expected "scons-options" "type" to be "array", but '
            'it was "{}"'.format(scons_options["type"]),
        )

        self.assertTrue(
            "minitems" in scons_options,
            'Expected "minitems" to be included in "scons-options"',
        )
        self.assertThat(
            scons_options["minitems"],
            Equals(1),
            'Expected "scons-options" "minitems" to be 1, but '
            'it was "{}"'.format(scons_options["minitems"]),
        )

        self.assertTrue(
            "uniqueItems" in scons_options,
            'Expected "uniqueItems" to be included in "scons-options"',
        )
        self.assertTrue(
            scons_options["uniqueItems"],
            'Expected "scons-options" "uniqueItems" to be "True"',
        )

    def test_get_build_properties(self):
        expected_build_properties = ["scons-options"]
        resulting_build_properties = scons.SconsPlugin.get_build_properties()

        self.assertThat(
            resulting_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)

    def scons_build(self):
        """Helper to call a full build"""
        plugin = scons.SconsPlugin("test-part", self.options, self.project_options)
        # Create fake scons
        plugin.build()
        return plugin

    @mock.patch.object(scons.SconsPlugin, "run")
    def test_build_with_destdir(self, run_mock):
        """Test building via scons and check for known calls and destdir"""
        plugin = self.scons_build()
        env = os.environ.copy()
        env["DESTDIR"] = plugin.installdir

        self.assertThat(run_mock.call_count, Equals(2))
        run_mock.assert_has_calls(
            [
                mock.call(["scons", "--debug=explain"]),
                mock.call(["scons", "install", "--debug=explain"], env=env),
            ]
        )
