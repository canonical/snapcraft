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
import pathlib
import sys
from textwrap import dedent

import fixtures
from testtools.matchers import Equals, IsInstance

from snapcraft.internal import errors
from snapcraft.plugins._plugin_finder import _PLUGINS
from snapcraft.plugins.v1 import PluginV1
from snapcraft.plugins.v2 import PluginV2
from tests import unit


class NonLocalTest(unit.TestCase):
    def test_unknown_plugin_must_raise_exception(self):
        path = copy.copy(sys.path)

        raised = self.assertRaises(
            errors.PluginError, self.load_part, "fake-part", "not-found"
        )

        self.assertThat(raised.message, Equals("unknown plugin: 'not-found'"))

        # Make sure that nothing was added to sys.path.
        self.assertThat(path, Equals(sys.path))

    def test_local_plugin(self):
        plugin_path = pathlib.Path("snap/plugins/x_local.py")
        plugin_path.parent.mkdir(parents=True)
        with open(plugin_path, "w") as plugin:
            print(
                dedent(
                    """\
                import snapcraft.plugins.v1
                class Local(snapcraft.plugins.v1.PluginV1):
                    pass
                """
                ),
                file=plugin,
            )

        self.load_part("test-part", plugin_name="x-local")


class InTreePluginsTest(unit.TestCase):
    def test_all_known_v1(self):
        # We don't want validation to take place here.
        self.useFixture(fixtures.MockPatch("jsonschema.validate"))
        for plugin_name in _PLUGINS["v1"]:
            plugin_handler = self.load_part(
                "test-part", plugin_name=plugin_name, base="core18"
            )
            self.expectThat(plugin_handler.plugin, IsInstance(PluginV1))

    def test_all_v2(self):
        self.useFixture(fixtures.MockPatch("jsonschema.validate"))
        for plugin_name in _PLUGINS["v2"]:
            plugin_handler = self.load_part(
                "test-part", plugin_name=plugin_name, base="core20"
            )
            self.expectThat(plugin_handler.plugin, IsInstance(PluginV2))

    def test_fail_on_schema(self):
        # conda requires some part_properties to be set.
        self.assertRaises(
            errors.PluginError,
            self.load_part,
            "test-part",
            plugin_name="conda",
            base="core18",
        )
