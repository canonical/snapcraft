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
import subprocess
from textwrap import dedent

import testscenarios
import yaml
from testtools.matchers import Contains, FileExists

from tests import integration


class VersionScriptBaseTestCase(integration.TestCase):
    def _set_version_script(self, snapcraft_yaml_file):
        with open(snapcraft_yaml_file) as f:
            snapcraft_yaml = yaml.load(f)
        snapcraft_yaml["version-script"] = self.scripts[self.script]
        with open(snapcraft_yaml_file, "w") as f:
            yaml.dump(snapcraft_yaml, f)


class VersionScriptTestCase(testscenarios.WithScenarios, VersionScriptBaseTestCase):

    scripts = {
        "empty": "",  # this is 0.1
        "simple": "echo from-version-script",
        "exported-version": (
            "echo from-variable-$SNAPCRAFT_PROJECT_VERSION"
        ),  # this is 0.1
        "exported-version-and-grade": (
            "echo $SNAPCRAFT_PROJECT_VERSION-$SNAPCRAFT_PROJECT_GRADE"
        ),
        "multi-line": dedent(
            """\
            if [ "$SNAPCRAFT_PROJECT_GRADE" = "devel" ] ; then
                echo "development-build"
            else
                exit 1
            fi
            """
        ),
        "cat-file": "cat stage/version",
    }

    scenarios = [
        ("empty", dict(script="empty", expected_version="0.1")),
        ("simple", dict(script="simple", expected_version="from-version-script")),
        (
            "version",
            dict(script="exported-version", expected_version="from-variable-0.1"),
        ),
        (
            "version-and-grade",
            dict(script="exported-version-and-grade", expected_version="0.1-devel"),
        ),
        ("multi-line", dict(script="multi-line", expected_version="development-build")),
        ("cat-file", dict(script="cat-file", expected_version="test-build")),
    ]

    def test_version(self):
        self.copy_project_to_cwd("version-script")
        self._set_version_script(os.path.join("snap", "snapcraft.yaml"))

        self.run_snapcraft("snap")

        self.assertThat(
            "version-script-test_{}_{}.snap".format(
                self.expected_version, self.deb_arch
            ),
            FileExists(),
        )


class VersionScriptErrorsTestCase(
    testscenarios.WithScenarios, VersionScriptBaseTestCase
):

    scripts = {"empty-version": "echo", "exit-2": "exit 2"}

    scenarios = (
        (
            "no output",
            dict(script="empty-version", error="The version-script produced no output"),
        ),
        (
            "exit 2",
            dict(
                script="exit-2", error="The version-script failed to run (exit code 2)"
            ),
        ),
    )

    def test_no_output(self):
        self.copy_project_to_cwd("version-script")
        self._set_version_script(os.path.join("snap", "snapcraft.yaml"))

        exception = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, ["snap"]
        )
        self.assertThat(exception.output, Contains(self.error))
