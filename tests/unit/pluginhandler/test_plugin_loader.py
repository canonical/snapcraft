# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018,2020 Canonical Ltd
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
from tests import unit


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
        import_mock.assert_called_with("snapcraft.plugins.v1.mock")
        local_load_mock.assert_called_with("x-mock", self.local_plugins_dir)
