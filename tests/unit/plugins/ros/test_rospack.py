# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

import subprocess

from unittest import mock
from testtools.matchers import Equals

from snapcraft.plugins._ros import rospack

import snapcraft
from tests import unit


class RospackTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.project = snapcraft.ProjectOptions()

        self.rospack = rospack.Rospack(
            ros_distro="kinetic",
            ros_package_path="package_path",
            rospack_path="rospack_path",
            ubuntu_sources="sources",
            ubuntu_keyrings=["keyring"],
            project=self.project,
        )

        patcher = mock.patch("snapcraft.repo.Ubuntu")
        self.ubuntu_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("subprocess.check_output")
        self.check_output_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_setup(self):
        # Return something other than a Mock to ease later assertions
        self.check_output_mock.return_value = b""

        self.rospack.setup()

        # Verify that only rospack and catkin were installed (no other .debs)
        self.assertThat(self.ubuntu_mock.call_count, Equals(1))
        self.assertThat(self.ubuntu_mock.return_value.get.call_count, Equals(1))
        self.assertThat(self.ubuntu_mock.return_value.unpack.call_count, Equals(1))
        self.ubuntu_mock.assert_has_calls(
            [
                mock.call(
                    self.rospack._rospack_path,
                    sources="sources",
                    keyrings=["keyring"],
                    project_options=self.project,
                ),
                mock.call().get(["ros-kinetic-rospack", "ros-kinetic-catkin"]),
                mock.call().unpack(self.rospack._rospack_install_path),
            ]
        )

    def test_setup_can_run_multiple_times(self):
        self.rospack.setup()

        # Make sure running setup() again doesn't have problems with the old
        # environment. An exception will be raised if setup can't be called twice.
        self.rospack.setup()

    def test_list_names(self):
        self.check_output_mock.return_value = b"foo\nbar\nbaz"

        self.assertThat(self.rospack.list_names(), Equals({"foo", "bar", "baz"}))

        self.check_output_mock.assert_called_with(
            ["/bin/bash", mock.ANY, "rospack", "list-names"], stderr=subprocess.STDOUT
        )

    def test_list_names_no_names(self):
        self.check_output_mock.return_value = b""

        self.assertThat(self.rospack.list_names(), Equals(set()))

        self.check_output_mock.assert_called_with(
            ["/bin/bash", mock.ANY, "rospack", "list-names"], stderr=subprocess.STDOUT
        )
