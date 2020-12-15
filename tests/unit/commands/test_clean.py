# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2020 Canonical Ltd
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

from testtools.matchers import Equals

from snapcraft.internal import steps

from . import LifecycleCommandsBaseTestCase


class CleanCommandTestCase(LifecycleCommandsBaseTestCase):
    def test_unprime_fails(self):
        result = self.run_command(["clean", "--unprime"])

        self.assertThat(result.exit_code, Equals(2))

    def test_using_defaults(self):
        result = self.run_command(["clean"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("multipass")
        self.fake_lifecycle_clean.mock.assert_called_once_with(
            mock.ANY, tuple(), steps.PRIME
        )
        self.provider_mock.clean_parts.assert_not_called()
        self.provider_class_mock().clean_project.assert_called_once_with()

    def test_with_parts_specified_using_defaults(self):
        result = self.run_command(["clean", "part0", "part1", "part2"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("multipass")
        self.fake_lifecycle_clean.mock.assert_not_called()
        self.provider_mock.clean_parts.assert_called_once_with(
            part_names=tuple(["part0", "part1", "part2"])
        )
        self.provider_class_mock().clean_project.assert_not_called()

    def test_using_lxd(self):
        result = self.run_command(["clean", "--use-lxd"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("lxd")
        self.fake_lifecycle_clean.mock.assert_called_once_with(
            mock.ANY, tuple(), steps.PRIME
        )
        self.provider_mock.clean_parts.assert_not_called()
        self.provider_class_mock().clean_project.assert_called_once_with()

    def test_with_parts_specified_using_lxd(self):
        result = self.run_command(["clean", "--use-lxd", "part0", "part1", "part2"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("lxd")
        self.fake_lifecycle_clean.mock.assert_not_called()
        self.provider_mock.clean_parts.assert_called_once_with(
            part_names=tuple(["part0", "part1", "part2"])
        )
        self.provider_class_mock().clean_project.assert_not_called()

    def test_using_destructive_mode(self):
        result = self.run_command(["clean", "--destructive-mode"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_not_called()
        self.fake_lifecycle_clean.mock.assert_called_once_with(mock.ANY, tuple(), None)

    def test_with_parts_specified_using_destructive_mode(self):
        result = self.run_command(
            ["clean", "--destructive-mode", "part0", "part1", "part2"]
        )

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_not_called()
        self.fake_lifecycle_clean.mock.assert_called_once_with(
            mock.ANY, tuple(["part0", "part1", "part2"]), None
        )
