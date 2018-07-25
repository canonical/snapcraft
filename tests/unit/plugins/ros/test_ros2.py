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

from unittest import mock

from snapcraft.plugins._ros import ros2

import snapcraft
from tests import unit


class Ros2TestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.project = snapcraft.ProjectOptions()

        self.bootstrapper = ros2.Bootstrapper(
            version="release-beta3",
            bootstrap_path="bootstrap_path",
            ubuntu_sources="sources",
            project=self.project,
        )

        patcher = mock.patch("snapcraft.repo.Ubuntu")
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("subprocess.check_call")
        self.check_call_mock = patcher.start()
        self.addCleanup(patcher.stop)

    @mock.patch("snapcraft.sources.Script")
    def test_pull(self, mock_script):
        self.bootstrapper.pull()

        # Assert that the ros2 repos file was pulled down
        mock_script.return_value.download.assert_called_once_with()

        # Verify that python3-vcstool is installed
        self.ubuntu_mock.assert_has_calls(
            [
                mock.call().get(["python3-vcstool"]),
                mock.call().unpack(self.bootstrapper._tool_dir),
            ]
        )

        # Verify that vcstool is then used to fetch the ROS2 underlay
        ros2_repos = os.path.join(self.bootstrapper._underlay_dir, "ros2.repos")
        self.check_call_mock.assert_called_once_with(
            ["vcs", "import", "--input", ros2_repos, self.bootstrapper._source_dir],
            env=mock.ANY,
        )

    def test_build(self):
        self.bootstrapper.build()

        ament_path = os.path.join(
            self.bootstrapper._source_dir, "ament", "ament_tools", "scripts", "ament.py"
        )
        self.check_call_mock.assert_called_once_with(
            [
                ament_path,
                "build",
                self.bootstrapper._source_dir,
                "--build-space",
                self.bootstrapper._build_dir,
                "--install-space",
                self.bootstrapper._install_dir,
            ],
            env=mock.ANY,
        )
