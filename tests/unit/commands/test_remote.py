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

from unittest import mock

import fixtures
from testtools.matchers import Contains, Equals

import snapcraft.internal.remote_build.errors as errors
from tests import fixture_setup

from . import CommandBaseTestCase


class RemoteBuildTests(CommandBaseTestCase):
    def setUp(self):
        super().setUp()
        self.snapcraft_yaml = fixture_setup.SnapcraftYaml(
            self.path, parts={"part0": {"plugin": "nil"}},
        )
        self.useFixture(self.snapcraft_yaml)

        self.mock_lc = self.useFixture(
            fixtures.MockPatch("snapcraft.cli.remote.LaunchpadClient", autospec=True)
        ).mock.return_value
        self.mock_lc_architectures = mock.PropertyMock(return_value=["i386"])
        type(self.mock_lc).architectures = self.mock_lc_architectures
        self.mock_lc.has_outstanding_build.return_value = False

    def test_remote_build(self):
        result = self.run_command(["remote-build", "--launchpad-accept-public-upload"])

        self.mock_lc.start_build.assert_called_once()
        self.mock_lc.cleanup.assert_called_once()
        self.assertThat(result.output, Contains("Building snap package for i386."))
        self.assertThat(result.exit_code, Equals(0))

    def test_remote_build_multiple_arch(self):
        self.mock_lc_architectures.return_value = ["i386", "amd64", "arm64"]

        result = self.run_command(
            [
                "remote-build",
                "--launchpad-accept-public-upload",
                "--build-on",
                "i386,amd64,arm64",
            ]
        )

        self.mock_lc.start_build.assert_called_once()
        self.mock_lc.cleanup.assert_called_once()
        self.assertThat(
            result.output, Contains("Building snap package for amd64, arm64, and i386.")
        )
        self.assertThat(result.exit_code, Equals(0))

    def test_remote_build_invalid_user_arch(self):
        self.assertRaises(
            errors.UnsupportedArchitectureError,
            self.run_command,
            ["remote-build", "--launchpad-accept-public-upload", "--build-on", "x64"],
        )

        self.mock_lc.start_build.assert_not_called()
        self.mock_lc.cleanup.assert_not_called()

    @mock.patch("snapcraft.cli.remote.echo")
    def test_remote_build_sudo_errors(self, mock_echo):
        self.useFixture(fixtures.EnvironmentVariable("SUDO_USER", "testuser"))
        self.useFixture(fixtures.MockPatch("os.geteuid", return_value=0))

        self.run_command(["remote-build", "--launchpad-accept-public-upload"])
        assert mock_echo.has_calls(
            [
                mock.call.warning(
                    "Running with 'sudo' may cause permission errors and is discouraged."
                )
            ]
        )
