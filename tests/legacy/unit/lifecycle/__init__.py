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

import logging
import textwrap

import fixtures

import snapcraft_legacy
from snapcraft_legacy.internal import project_loader
from tests.legacy import unit


class LifecycleTestBase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.project_options = snapcraft_legacy.ProjectOptions()

        self.fake_install_build_packages = fixtures.MockPatch(
            "snapcraft_legacy.internal.lifecycle._runner._install_build_packages",
            return_value=list(),
        )
        self.useFixture(self.fake_install_build_packages)

        self.useFixture(
            fixtures.MockPatch(
                "snapcraft_legacy.internal.project_loader._config.Config.get_build_packages",
                return_value=set(),
            )
        )

        self.fake_install_build_snaps = fixtures.MockPatch(
            "snapcraft_legacy.internal.lifecycle._runner._install_build_snaps",
            return_value=list(),
        )
        self.useFixture(self.fake_install_build_snaps)

        self.useFixture(
            fixtures.MockPatch(
                "snapcraft_legacy.internal.project_loader._config.Config.get_build_snaps",
                return_value=set(),
            )
        )

    def make_snapcraft_project(self, parts, snap_type=""):
        yaml = textwrap.dedent(
            """\
            name: test
            base: core20
            version: "1.0"
            summary: test
            description: test
            confinement: strict
            grade: stable
            {type}

            {parts}
            """
        )

        self.snapcraft_yaml_file_path = self.make_snapcraft_yaml(
            yaml.format(parts=parts, type=snap_type)
        )
        project = snapcraft_legacy.project.Project(
            snapcraft_yaml_file_path=self.snapcraft_yaml_file_path
        )
        return project_loader.load_config(project)
