# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License remote_build 3 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

from testtools.matchers import Contains, Equals
from textwrap import dedent

import snapcraft.internal.remote_build.errors as errors
from . import CommandBaseTestCase
from unittest import mock


class RemoteBuildTests(CommandBaseTestCase):
    def setUp(self):
        super().setUp()

    def create_project(self):
        snapcraft_yaml = dedent(
            """\
            name: test
            base: core18
            version: "1"
            summary: test
            description: test
            confinement: strict
            grade: stable

            parts:
              part1:
                plugin: nil
        """
        )
        self.make_snapcraft_yaml(content=snapcraft_yaml)

    @mock.patch("snapcraft.cli.remote.LaunchpadClient")
    def test_remote_build(self, mock_lc):
        lc = mock_lc.return_value
        lc.architectures = ["i386"]
        lc.has_outstanding_build = mock.Mock()
        lc.has_outstanding_build.return_value = False
        lc.start_build = mock.Mock()
        lc.cleanup = mock.Mock()
        self.create_project()

        result = self.run_command(
            ["remote-build", "--accept-public-upload", "--user", "testuser"]
        )

        lc.start_build.assert_called_once()
        lc.cleanup.assert_called_once()
        self.assertThat(result.output, Contains("Building snap package for i386."))
        self.assertThat(result.exit_code, Equals(0))

    @mock.patch("snapcraft.cli.remote.LaunchpadClient")
    def test_remote_build_multiple_arch(self, mock_lc):
        lc = mock_lc.return_value
        lc.architectures = ["i386", "amd64", "arm64"]
        lc.has_outstanding_build = mock.Mock()
        lc.has_outstanding_build.return_value = False
        lc.start_build = mock.Mock()
        lc.cleanup = mock.Mock()
        self.create_project()

        result = self.run_command(
            [
                "remote-build",
                "--accept-public-upload",
                "--user",
                "testuser",
                "--arch",
                "i386,amd64,arm64",
            ]
        )

        lc.start_build.assert_called_once()
        lc.cleanup.assert_called_once()
        self.assertThat(
            result.output, Contains("Building snap package for amd64, arm64, and i386.")
        )
        self.assertThat(result.exit_code, Equals(0))

    @mock.patch("snapcraft.cli.remote.LaunchpadClient")
    def test_remote_build_invalid_user_arch(self, mock_lc):
        lc = mock_lc.return_value
        lc.has_outstanding_build = mock.Mock()
        lc.has_outstanding_build.return_value = False
        lc.start_build = mock.Mock()
        lc.cleanup = mock.Mock()
        self.create_project()

        self.assertRaises(
            errors.UnsupportedArchitectureError,
            self.run_command,
            [
                "remote-build",
                "--accept-public-upload",
                "--user",
                "testuser",
                "--arch",
                "x64",
            ],
        )

        lc.start_build.assert_not_called()
        lc.cleanup.assert_not_called()

    @mock.patch("snapcraft.cli.remote.LaunchpadClient")
    def test_clean(self, mock_lc):
        lc = mock_lc.return_value
        lc.cleanup = mock.Mock()
        self.create_project()

        result = self.run_command(
            ["remote-build", "--accept-public-upload", "--user", "testuser", "--clean"]
        )

        lc.cleanup.assert_called_once()
        self.assertThat(result.output, Contains("Cleaning..."))
        self.assertThat(result.exit_code, Equals(0))
