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

from snapcraft_legacy.internal import errors
from snapcraft_legacy.plugins._plugin_finder import _PLUGINS
from snapcraft_legacy.plugins.v2 import PluginV2
from tests.legacy import unit


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
                import snapcraft_legacy.plugins.v2
                class PluginImpl(snapcraft_legacy.plugins.v2.PluginV2):
                    @classmethod
                    def get_schema(cls):
                        return {}

                    def get_build_commands(self):
                        return []

                    def get_build_environment(self):
                        return []

                    def get_build_packages(self):
                        return []

                    def get_build_snaps(self):
                        return []
                """
                ),
                file=plugin,
            )

        self.load_part("test-part", plugin_name="x-local")


class InTreePluginsTest(unit.TestCase):
    def test_all_v2(self):
        self.useFixture(fixtures.MockPatch("jsonschema.validate"))
        for plugin_name in _PLUGINS:
            plugin_handler = self.load_part(
                "test-part", plugin_name=plugin_name, base="core20"
            )
            self.expectThat(plugin_handler.plugin, IsInstance(PluginV2))
