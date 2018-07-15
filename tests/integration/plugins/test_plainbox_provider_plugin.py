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
import subprocess

import testscenarios
from testtools.matchers import FileExists

from tests import integration


class PlainboxProviderPluginStageTestCase(
    testscenarios.WithScenarios, integration.TestCase
):

    scenarios = [
        ("basic", dict(project_directory="plainbox-provider")),
        (
            "with-stage-packages",
            dict(project_directory="plainbox-provider-with-stage-packages"),
        ),
    ]

    def test_stage(self):
        self.run_snapcraft("stage", self.project_directory)

        self.assertThat(
            os.path.join(
                self.stage_dir,
                "providers",
                "simple-plainbox-provider",
                "plainbox-provider-simple.provider",
            ),
            FileExists(),
        )


class PlainboxProviderPluginTestCase(integration.TestCase):
    def test_snap_provider_with_deps(self):
        project_dir = "plainbox-provider-with-deps"
        self.run_snapcraft("stage", project_dir)
        # No assertion required as the project will fail to complete to the
        # prime step if validation fails

    def test_invalid_provider(self):
        project_dir = "plainbox-provider-invalid"
        # The validate command should fail during the builid step of the
        # provider in this project
        self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, "prime", project_dir
        )

    def test_python_stage_pkg_mix(self):
        project_dir = "plainbox-provider-python-stage-pkg-mix"
        self.run_snapcraft("stage", project_dir)
        # Tests bug where snap would fail to build with error:
        # "Failed building wheel for bitstring"
