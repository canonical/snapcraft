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

import copy
import logging
import sys
from unittest.mock import patch

import fixtures
from testtools.matchers import Equals

import snapcraft
from snapcraft.internal import errors
from tests import fixture_setup, unit


class PluginLoaderTestCase(unit.TestCase):
    def test_unknown_plugin_must_raise_exception(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        path = copy.copy(sys.path)

        raised = self.assertRaises(
            errors.PluginError, self.load_part, "fake-part", "test_unexisting"
        )

        self.assertThat(raised.message, Equals("unknown plugin: 'test_unexisting'"))

        # Make sure that nothing was added to sys.path.
        self.assertThat(path, Equals(sys.path))

    def test_known_module_but_unknown_plugin_must_raise_exception(self):
        fake_logger = fixtures.FakeLogger(level=logging.ERROR)
        self.useFixture(fake_logger)

        path = copy.copy(sys.path)

        # "_ros" is a valid module within the plugin path, but contains no
        # plugins.
        raised = self.assertRaises(
            errors.PluginError, self.load_part, "fake-part", "_ros"
        )

        self.assertThat(raised.message, Equals("no plugin found in module '_ros'"))

        # Make sure that nothing was added to sys.path.
        self.assertThat(path, Equals(sys.path))

    @patch("importlib.import_module")
    @patch("snapcraft.internal.pluginhandler._plugin_loader._load_local")
    @patch("snapcraft.internal.pluginhandler._plugin_loader._get_plugin")
    def test_non_local_plugins(self, plugin_mock, local_load_mock, import_mock):
        class TestPlugin(snapcraft.BasePlugin):
            def __init__(self, name, options, project):
                super().__init__(name, options, project)

        plugin_mock.return_value = TestPlugin
        local_load_mock.side_effect = ImportError()
        self.load_part("mock-part", "mock")
        import_mock.assert_called_with("snapcraft.plugins.mock")
        local_load_mock.assert_called_with("x-mock", self.local_plugins_dir)

    def test_plugin_without_project(self):
        class OldPlugin(snapcraft.BasePlugin):
            @classmethod
            def schema(cls):
                schema = super().schema()
                schema["properties"]["fake-property"] = {"type": "string"}
                return schema

            def __init__(self, name, options):
                super().__init__(name, options)

        self.useFixture(fixture_setup.FakePlugin("oldplugin", OldPlugin))
        handler = self.load_part("fake-part", "oldplugin", {"fake-property": "."})

        self.assertTrue(handler.plugin.project is not None)

    @patch("importlib.import_module")
    @patch("snapcraft.internal.pluginhandler._plugin_loader._load_local")
    @patch("snapcraft.internal.pluginhandler._plugin_loader._get_plugin")
    def test_plugin_without_project_not_from_base(
        self, plugin_mock, local_load_mock, import_mock
    ):
        class NonBaseOldPlugin:
            @classmethod
            def schema(cls):
                return {}

            @classmethod
            def get_pull_properties(cls):
                return []

            @classmethod
            def get_build_properties(cls):
                return []

            def __init__(self, name, options):
                self.name = "old_plugin"
                self.osrepodir = "osrepodir"
                self.statedir = "statedir"
                self.sourcedir = "sourcedir"
                self.build_basedir = "build_basedir"

            def build(self):
                pass

        plugin_mock.return_value = NonBaseOldPlugin
        local_load_mock.side_effect = ImportError()
        handler = self.load_part("fake-part", "nonbaseoldplugin")

        self.assertTrue(handler.plugin.project is not None)

    def test_plugin_schema_step_hint_pull(self):
        class Plugin(snapcraft.BasePlugin):
            @classmethod
            def schema(cls):
                schema = super().schema()
                schema["properties"]["foo"] = {"type": "string"}
                schema["pull-properties"] = ["foo"]

                return schema

        self.useFixture(fixture_setup.FakePlugin("plugin", Plugin))
        self.load_part("fake-part", "plugin")

    def test_plugin_schema_step_hint_build(self):
        class Plugin(snapcraft.BasePlugin):
            @classmethod
            def schema(cls):
                schema = super().schema()
                schema["properties"]["foo"] = {"type": "string"}
                schema["build-properties"] = ["foo"]

                return schema

        self.useFixture(fixture_setup.FakePlugin("plugin", Plugin))
        self.load_part("fake-part", "plugin")

    def test_plugin_schema_step_hint_pull_and_build(self):
        class Plugin(snapcraft.BasePlugin):
            @classmethod
            def schema(cls):
                schema = super().schema()
                schema["properties"]["foo"] = {"type": "string"}
                schema["pull-properties"] = ["foo"]
                schema["build-properties"] = ["foo"]

                return schema

        self.useFixture(fixture_setup.FakePlugin("plugin", Plugin))
        self.load_part("fake-part", "plugin")

    def test_plugin_schema_invalid_pull_hint(self):
        class Plugin(snapcraft.BasePlugin):
            @classmethod
            def schema(cls):
                schema = super().schema()
                schema["properties"]["foo"] = {"type": "string"}
                schema["pull-properties"] = ["bar"]

                return schema

        self.useFixture(fixture_setup.FakePlugin("plugin", Plugin))
        raised = self.assertRaises(
            errors.InvalidPullPropertiesError, self.load_part, "fake-part", "plugin"
        )

        self.assertThat(raised.plugin_name, Equals("plugin"))
        self.assertThat(raised.properties, Equals(["bar"]))

    def test_plugin_schema_invalid_build_hint(self):
        class Plugin(snapcraft.BasePlugin):
            @classmethod
            def schema(cls):
                schema = super().schema()
                schema["properties"]["foo"] = {"type": "string"}
                schema["build-properties"] = ["bar"]

                return schema

        self.useFixture(fixture_setup.FakePlugin("plugin", Plugin))
        raised = self.assertRaises(
            errors.InvalidBuildPropertiesError, self.load_part, "fake-part", "plugin"
        )

        self.assertThat(raised.plugin_name, Equals("plugin"))
        self.assertThat(raised.properties, Equals(["bar"]))
