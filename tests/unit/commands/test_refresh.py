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
from textwrap import dedent
from unittest import mock

from testtools.matchers import Equals

from tests.unit import TestWithFakeRemoteParts

from . import CommandBaseTestCase


class RefreshCommandBaseTestCase(CommandBaseTestCase, TestWithFakeRemoteParts):

    yaml_template = dedent(
        """\
        name: snap-test
        version: "1.0"
        summary: test snapping
        description: if snap is successful a snap package will be available
        architectures: ['amd64']
        type: {}
        confinement: strict
        grade: stable

        parts:
            part1:
                plugin: nil
        """
    )

    def make_snapcraft_yaml(self, n=1, snap_type="app", snapcraft_yaml=None):
        if not snapcraft_yaml:
            snapcraft_yaml = self.yaml_template.format(snap_type)
        super().make_snapcraft_yaml(snapcraft_yaml)
        self.state_dir = os.path.join(self.parts_dir, "part1", "state")


class RefreshCommandTestCase(RefreshCommandBaseTestCase):
    @mock.patch("snapcraft.cli.containers.repo.Repo.refresh_build_packages")
    def test_refresh(self, mock_repo_refresh):
        self.make_snapcraft_yaml()

        result = self.run_command(["refresh"])

        self.assertThat(result.exit_code, Equals(0))
        mock_repo_refresh.assert_called_once_with()
