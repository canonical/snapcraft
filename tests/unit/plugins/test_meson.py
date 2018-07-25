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

from unittest import mock
from testtools.matchers import Equals, HasLength

import snapcraft
from snapcraft.plugins import meson
from tests import unit


class MesonPluginTestCase(unit.TestCase):
    """Plugin to provide snapcraft support for the meson build system"""

    def setUp(self):
        super(MesonPluginTestCase, self).setUp()

        class Options:
            """Internal Options Class matching the Meson plugin"""

            meson_parameters = []

        self.options = Options()
        self.project_options = snapcraft.ProjectOptions()

    def test_schema(self):
        """Test validity of the Meson Plugin schema"""
        schema = meson.MesonPlugin.schema()

        # Verify the presence of all properties
        properties = schema["properties"]
        self.assertTrue(
            "meson-parameters" in properties,
            'Expected "meson-parameters" to be included in ' "properties",
        )

        meson_parameters = properties["meson-parameters"]

        self.assertTrue(
            "type" in meson_parameters,
            'Expected "type" to be included in "meson-parameters"',
        )
        self.assertThat(
            meson_parameters["type"],
            Equals("array"),
            'Expected "meson-parameters" "type" to be "array", '
            'but it was "{}"'.format(meson_parameters["type"]),
        )

        self.assertTrue(
            "minitems" in meson_parameters,
            'Expected "minitems" to be included in "meson-parameters"',
        )
        self.assertThat(
            meson_parameters["minitems"],
            Equals(1),
            'Expected "meson-parameters" "minitems" to be 1, but '
            'it was "{}"'.format(meson_parameters["minitems"]),
        )

        self.assertTrue(
            "uniqueItems" in meson_parameters,
            'Expected "uniqueItems" to be included in "meson-parameters"',
        )
        self.assertTrue(
            meson_parameters["uniqueItems"],
            'Expected "meson-parameters" "uniqueItems" to be "True"',
        )

    def test_get_build_properties(self):
        expected_build_properties = ["meson-parameters"]
        resulting_build_properties = meson.MesonPlugin.get_build_properties()

        self.assertThat(
            resulting_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)

    @mock.patch.object(meson.MesonPlugin, "run")
    def test_build(self, run_mock):
        """Test building via meson and check for known calls and destdir"""
        plugin = meson.MesonPlugin("test-part", self.options, self.project_options)

        plugin.build()

        env = os.environ.copy()
        env["DESTDIR"] = plugin.installdir

        self.assertThat(run_mock.call_count, Equals(3))
        run_mock.assert_has_calls(
            [
                mock.call(["meson", plugin.snapbuildname]),
                mock.call(["ninja"], cwd=plugin.mesonbuilddir),
                mock.call(["ninja", "install"], env=env, cwd=plugin.mesonbuilddir),
            ]
        )

    @mock.patch.object(meson.MesonPlugin, "run")
    def test_build_with_parameters(self, run_mock):
        """Test with parameters"""
        self.options.meson_parameters = ["--strip"]
        plugin = meson.MesonPlugin("test-part", self.options, self.project_options)

        plugin.build()

        env = os.environ.copy()
        env["DESTDIR"] = plugin.installdir

        self.assertThat(run_mock.call_count, Equals(3))
        run_mock.assert_has_calls(
            [
                mock.call(["meson", "--strip", plugin.snapbuildname]),
                mock.call(["ninja"], cwd=plugin.mesonbuilddir),
                mock.call(["ninja", "install"], env=env, cwd=plugin.mesonbuilddir),
            ]
        )
