# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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
from testtools.matchers import Equals, DirExists, Not
import snapcraft.internal.errors

from . import LifecycleCommandsBaseTestCase


class PullCommandTestCase(LifecycleCommandsBaseTestCase):
    def test_pull_invalid_part(self):
        self.make_snapcraft_yaml("pull")

        raised = self.assertRaises(
            snapcraft.internal.errors.SnapcraftEnvironmentError,
            self.run_command,
            ["pull", "no-pull"],
        )

        self.assertThat(
            str(raised),
            Equals("The part named 'no-pull' is not defined in 'snap/snapcraft.yaml'"),
        )

    def test_pull_defaults(self):
        parts = self.make_snapcraft_yaml("pull")

        result = self.run_command(["pull"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(self.parts_dir, DirExists())
        self.assertThat(parts[0]["part_dir"], DirExists())

        self.verify_state("pull0", parts[0]["state_dir"], "pull")

    def test_pull_one_part_only_from_3(self):
        parts = self.make_snapcraft_yaml("pull", n=3)

        result = self.run_command(["pull", "pull1"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(self.parts_dir, DirExists())
        self.assertThat(parts[1]["part_dir"], DirExists())

        self.verify_state("pull1", parts[1]["state_dir"], "pull")

        for i in [0, 2]:
            self.assertThat(parts[i]["part_dir"], Not(DirExists()))
            self.assertThat(parts[i]["state_dir"], Not(DirExists()))

    @mock.patch("snapcraft.repo.Repo.get")
    @mock.patch("snapcraft.repo.Repo.unpack")
    def test_pull_stage_packages_without_geoip(self, mock_unpack, mock_get):
        yaml_part = """  {step}{iter:d}:
        plugin: nil
        stage-packages: ['mir']"""
        self.make_snapcraft_yaml("pull", n=3, yaml_part=yaml_part)

        mock_get.return_value = "[mir=0.0]"

        result = self.run_command(["pull", "pull1"])

        self.assertThat(result.exit_code, Equals(0))

    @mock.patch("snapcraft.repo.Repo.get")
    @mock.patch("snapcraft.repo.Repo.unpack")
    def test_pull_stage_packages_with_geoip(self, mock_unpack, mock_get):
        yaml_part = """  {step}{iter:d}:
        plugin: nil
        stage-packages: ['mir']"""
        self.make_snapcraft_yaml("pull", n=3, yaml_part=yaml_part)

        mock_get.return_value = "[mir=0.0]"

        result = self.run_command(["pull", "pull1", "--enable-geoip"])

        self.assertThat(result.exit_code, Equals(0))

    @mock.patch("snapcraft.repo.Repo.get")
    @mock.patch("snapcraft.repo.Repo.unpack")
    def test_pull_multiarch_stage_package(self, mock_unpack, mock_get):
        yaml_part = """  {step}{iter:d}:
        plugin: nil
        stage-packages: ['mir:arch']"""
        self.make_snapcraft_yaml("pull", n=3, yaml_part=yaml_part)

        mock_get.return_value = "[mir=0.0]"

        result = self.run_command(["pull", "pull1"])

        self.assertThat(result.exit_code, Equals(0))
        mock_get.assert_called_once_with({"mir:arch"})
