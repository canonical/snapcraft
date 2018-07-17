# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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

import testscenarios
from testtools.matchers import FileExists

from tests import integration


class LocalPluginTestCase(integration.TestCase):
    def test_stage_local_plugin(self):
        self.run_snapcraft("stage", "local-plugin")

        self.assertThat(os.path.join(self.stage_dir, "build-stamp"), FileExists())

    def test_stage_local_plugin_in_parts(self):
        self.run_snapcraft("stage", "local-plugin-in-parts")

        self.assertThat(os.path.join(self.stage_dir, "build-stamp"), FileExists())


class LocalPluginCleanTestCase(testscenarios.WithScenarios, integration.TestCase):

    scenarios = [
        ("local-plugin", dict(project="local-plugin", base_dir="snap")),
        (
            "local-plugin-in-parts",
            dict(project="local-plugin-in-parts", base_dir="parts"),
        ),
    ]

    def test_clean(self):
        self.copy_project_to_cwd(self.project)
        self.run_snapcraft("stage")

        # Now clean, and verify that the local plugin is still there.
        self.run_snapcraft("clean")

        self.assertThat(
            os.path.join(self.base_dir, "plugins", "x_local_plugin.py"),
            FileExists(),
            "Expected local plugin to remain when cleaned",
        )
