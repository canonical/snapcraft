# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import snapcraft.plugins.jhbuild
from tests import unit


# LP: #1733584
class JHBuildPluginTestCase(unit.TestCase):  # type: ignore
    def _test_plugin(self):
        class Options:
            modules = ["gtk+"]
            snap_name = "test"

        return snapcraft.plugins.jhbuild.JHBuildPlugin("test", Options())

    def test_schema(self):
        schema = snapcraft.plugins.jhbuild.JHBuildPlugin.schema()

        for key in schema["properties"]:
            self.assertIn("type", schema["properties"][key])

        for key in schema["required"]:
            self.assertIn(key, schema["properties"])

        for key in schema["pull-properties"]:
            self.assertIn(key, schema["properties"])

        for key in schema["build-properties"]:
            self.assertIn(key, schema["properties"])

    def test_run_with_env(self):
        plugin = self._test_plugin()

        plugin.run(["printenv", "ANSWER"], env={"ANSWER": "42"})

        self.assertEqual(
            "42", plugin.run_output(["printenv", "ANSWER"], env={"ANSWER": "42"})
        )
