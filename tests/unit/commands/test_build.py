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

from unittest import mock

import fixtures
from testtools.matchers import Equals, DirExists, Not

from . import LifecycleCommandsBaseTestCase
import snapcraft.internal.errors


class BuildCommandTestCase(LifecycleCommandsBaseTestCase):
    def test_build_invalid_part(self):
        self.make_snapcraft_yaml("build")

        raised = self.assertRaises(
            snapcraft.internal.errors.SnapcraftEnvironmentError,
            self.run_command,
            ["build", "no-build"],
        )

        self.assertThat(
            str(raised),
            Equals("The part named 'no-build' is not defined in 'snap/snapcraft.yaml'"),
        )

    def test_build_defaults(self):
        parts = self.make_snapcraft_yaml("build")

        result = self.run_command(["build"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(self.parts_dir, DirExists())
        self.assertThat(parts[0]["part_dir"], DirExists())

        self.verify_state("build0", parts[0]["state_dir"], "build")

    def test_build_with_lxd_build_environment(self):
        self.make_snapcraft_yaml("build", n=2)

        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT", "lxd")
        )

        patcher = mock.patch("snapcraft.internal.lxd.Project")
        lxd_project_mock = patcher.start()
        self.addCleanup(patcher.stop)

        result = self.run_command(["build"])

        self.assertThat(result.exit_code, Equals(0))
        lxd_project_mock.assert_called_once_with(
            project=mock.ANY, source=".", output=None
        )
        lxd_project_mock().execute.assert_called_once_with("build", ())

    def test_build_part_with_lxd_build_environment(self):
        self.make_snapcraft_yaml("build", n=2)

        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT", "lxd")
        )

        patcher = mock.patch("snapcraft.internal.lxd.Project")
        lxd_project_mock = patcher.start()
        self.addCleanup(patcher.stop)

        result = self.run_command(["build", "build1"])

        self.assertThat(result.exit_code, Equals(0))
        lxd_project_mock.assert_called_once_with(
            project=mock.ANY, source=".", output=None
        )
        lxd_project_mock().execute.assert_called_once_with("build", ("build1",))

    def test_build_one_part_only_from_3(self):
        parts = self.make_snapcraft_yaml("build", n=3)

        result = self.run_command(["build", "build1"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(self.parts_dir, DirExists())
        self.assertThat(parts[1]["part_dir"], DirExists())

        self.verify_state("build1", parts[1]["state_dir"], "build")

        for i in [0, 2]:
            self.assertThat(parts[i]["part_dir"], Not(DirExists()))
            self.assertThat(parts[i]["state_dir"], Not(DirExists()))
