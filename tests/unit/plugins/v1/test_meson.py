# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018, 2020 Canonical Ltd
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
import subprocess
from unittest import mock

from testtools.matchers import Equals, HasLength

from snapcraft.internal import errors
from snapcraft.plugins.v1 import meson
from tests import unit

from . import PluginsV1BaseTestCase


class MesonPluginPropertiesTest(unit.TestCase):
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

    def test_get_pull_properties(self):
        expected_pull_properties = ["meson-version"]
        resulting_pull_properties = meson.MesonPlugin.get_pull_properties()

        self.assertThat(
            resulting_pull_properties, HasLength(len(expected_pull_properties))
        )

        for property in expected_pull_properties:
            self.assertIn(property, resulting_pull_properties)

    def test_get_build_properties(self):
        expected_build_properties = ["meson-parameters"]
        resulting_build_properties = meson.MesonPlugin.get_build_properties()

        self.assertThat(
            resulting_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)


class MesonPluginBaseTest(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        class Options:
            """Internal Options Class matching the Meson plugin"""

            meson_parameters = []
            meson_version = []

        self.options = Options()

    @mock.patch("subprocess.check_call")
    def test_pull(self, check_call_mock):
        """Test pulling with the default settings."""
        plugin = meson.MesonPlugin("test-part", self.options, self.project)

        plugin.pull()

        check_call_mock.assert_called_once_with(
            ["python3", "-m", "pip", "install", "-U", "meson"]
        )

    @mock.patch("subprocess.check_call")
    def test_pull_meson_version(self, check_call_mock):
        """Test pulling with the default settings."""
        self.options.meson_version = "1.0"
        plugin = meson.MesonPlugin("test-part", self.options, self.project)

        plugin.pull()

        check_call_mock.assert_called_once_with(
            ["python3", "-m", "pip", "install", "-U", "meson==1.0"]
        )

    @mock.patch(
        "subprocess.check_call",
        side_effect=subprocess.CalledProcessError(cmd=["meson-install"], returncode=1),
    )
    def test_pull_meson_installation_fails(self, check_call_mock):
        """Test pulling with the default settings."""
        self.options.meson_version = "1.0"
        plugin = meson.MesonPlugin("test-part", self.options, self.project)

        self.assertRaises(errors.SnapcraftPluginCommandError, plugin.pull)

    @mock.patch.object(meson.MesonPlugin, "run")
    def test_build(self, run_mock):
        """Test building via meson and check for known calls and destdir"""
        plugin = meson.MesonPlugin("test-part", self.options, self.project)

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
        plugin = meson.MesonPlugin("test-part", self.options, self.project)

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


class MesonPluginUnsupportedBase(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        self.project._snap_meta.base = "unsupported-base"

        class Options:
            source = "dir"

        self.options = Options()

    def test_unsupported_base_raises(self):
        self.assertRaises(
            errors.PluginBaseError,
            meson.MesonPlugin,
            "test-part",
            self.options,
            self.project,
        )
