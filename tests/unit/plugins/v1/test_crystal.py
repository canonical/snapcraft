# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019-2020 Canonical Ltd
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

import fixtures
from testtools.matchers import Equals, FileExists, HasLength

from snapcraft.internal import errors
from snapcraft.plugins.v1 import crystal
from tests import unit

from . import PluginsV1BaseTestCase


class CrystalPluginBaseTest(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        self.fake_run = fixtures.MockPatch("snapcraft.internal.common.run")
        self.useFixture(self.fake_run)


class CrystalPluginPropertiesTest(unit.TestCase):
    def test_schema(self):
        schema = crystal.CrystalPlugin.schema()

        properties = schema["properties"]
        for expected in ["crystal-channel"]:
            self.assertTrue(
                expected in properties,
                "Expected {!r} to be included in properties".format(expected),
            )

        # Check crystal-channel
        crystal_channel = properties["crystal-channel"]
        for expected in ["type", "default"]:
            self.assertTrue(
                expected in crystal_channel,
                "Expected {!r} to be included in 'crystal-channel'".format(expected),
            )

        crystal_channel_type = crystal_channel["type"]
        self.assertThat(
            crystal_channel_type,
            Equals("string"),
            'Expected "crystal-channel" "type" to be "string", but '
            'it was "{}"'.format(crystal_channel_type),
        )

        crystal_channel_default = crystal_channel["default"]
        self.assertThat(
            crystal_channel_default,
            Equals("latest/stable"),
            'Expected "crystal-channel" "default" to be '
            '"latest/stable", but it was "{}"'.format(crystal_channel_default),
        )

        # Check crystal-build-options
        crystal_build_options = properties["crystal-build-options"]
        for expected in ["type", "default"]:
            self.assertTrue(
                expected in crystal_channel,
                "Expected {!r} to be included in 'crystal-build-options'".format(
                    expected
                ),
            )

        crystal_build_options_type = crystal_channel["type"]
        self.assertThat(
            crystal_build_options_type,
            Equals("string"),
            'Expected "crystal-build-options" "type" to be "string", but '
            'it was "{}"'.format(crystal_build_options_type),
        )

        crystal_build_options_default = crystal_build_options["default"]
        self.assertThat(
            crystal_build_options_default,
            Equals([]),
            'Expected "crystal-build-options" "default" to be '
            '"[]", but it was "{}"'.format(crystal_build_options_default),
        )

        # Check required properties
        self.assertThat(schema["required"], Equals(["source"]))

    def test_get_pull_properties(self):
        expected_pull_properties = ["crystal-channel"]
        resulting_pull_properties = crystal.CrystalPlugin.get_pull_properties()

        self.assertThat(
            resulting_pull_properties, HasLength(len(expected_pull_properties))
        )

        for property in expected_pull_properties:
            self.assertIn(property, resulting_pull_properties)

    def test_get_build_properties(self):
        expected_build_properties = ["crystal-build-options"]
        resulting_build_properties = crystal.CrystalPlugin.get_build_properties()

        self.assertThat(
            resulting_build_properties, HasLength(len(expected_build_properties))
        )

        for property in expected_build_properties:
            self.assertIn(property, resulting_build_properties)


class MockElfFile:
    def __init__(self, *, path: str) -> None:
        self.path = path
        self.is_dynamic = True

    def load_dependencies(self, *args, **kwargs):
        return []


class CrystalPluginTest(CrystalPluginBaseTest):
    def test_pull(self):
        class Options:
            source = "dir"
            crystal_channel = "latest/stable"
            crystal_build_options = []

        plugin = crystal.CrystalPlugin("test-part", Options(), self.project)

        plugin.pull()

        self.fake_run.mock.assert_not_called()

    def test_build(self):
        class Options:
            source = "dir"
            crystal_channel = "latest/stable"
            crystal_build_options = []

        plugin = crystal.CrystalPlugin("test-part", Options(), self.project)

        # fake binaries being built
        self.useFixture(
            fixtures.MockPatch(
                "snapcraft.internal.elf.ElfFile", side_effect=MockElfFile
            )
        )
        binaries = ["foo", "bar"]
        bin_dir = os.path.join(plugin.builddir, "bin")
        os.makedirs(bin_dir)
        for b in binaries:
            open(os.path.join(bin_dir, b), "w").close()

        plugin.build()

        self.fake_run.mock.assert_has_calls(
            [
                mock.call(["shards", "install", "--production"], cwd=plugin.builddir),
                mock.call(["shards", "build", "--production"], cwd=plugin.builddir),
            ]
        )
        self.assertThat(self.fake_run.mock.call_count, Equals(2))

        for b in binaries:
            self.assertThat(os.path.join(plugin.installdir, "bin", b), FileExists())

    def test_build_flags(self):
        class Options:
            source = "dir"
            crystal_channel = "latest/stable"
            crystal_build_options = ["-Dpreview_mt"]

        plugin = crystal.CrystalPlugin("test-part", Options(), self.project)

        # fake binaries being built
        self.useFixture(
            fixtures.MockPatch(
                "snapcraft.internal.elf.ElfFile", side_effect=MockElfFile
            )
        )
        binaries = ["foo", "bar"]
        bin_dir = os.path.join(plugin.builddir, "bin")
        os.makedirs(bin_dir)
        for b in binaries:
            open(os.path.join(bin_dir, b), "w").close()

        plugin.build()

        self.fake_run.mock.assert_has_calls(
            [
                mock.call(["shards", "install", "--production"], cwd=plugin.builddir),
                mock.call(
                    ["shards", "build", "--production", "-Dpreview_mt"],
                    cwd=plugin.builddir,
                ),
            ]
        )
        self.assertThat(self.fake_run.mock.call_count, Equals(2))
        for b in binaries:
            self.assertThat(os.path.join(plugin.installdir, "bin", b), FileExists())

    def test_build_no_targets(self):
        class Options:
            source = "dir"
            crystal_channel = "latest/stable"
            crystal_build_options = []

        plugin = crystal.CrystalPlugin("test-part", Options(), self.project)

        self.assertRaises(errors.SnapcraftEnvironmentError, plugin.build)


class CrystalPluginUnsupportedBase(PluginsV1BaseTestCase):
    def setUp(self):
        super().setUp()

        self.project._snap_meta.base = "unsupported-base"

        class Options:
            source = "dir"
            crystal_channel = "latest/stable"
            crystal_build_options = []

        self.options = Options()

    def test_unsupported_base_raises(self):
        self.assertRaises(
            errors.PluginBaseError,
            crystal.CrystalPlugin,
            "test-part",
            self.options,
            self.project,
        )
