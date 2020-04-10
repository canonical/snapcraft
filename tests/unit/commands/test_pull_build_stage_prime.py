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


class TestPullBuildStagePrimeCommand(LifecycleCommandsBaseTestCase):
    scenarios = (
        ("pull", dict(step=steps.PULL, previous_step=None)),
        ("build", dict(step=steps.BUILD, previous_step=steps.PULL)),
        ("stage", dict(step=steps.STAGE, previous_step=steps.BUILD)),
        ("prime", dict(step=steps.PRIME, previous_step=steps.STAGE)),
    )

    def assert_build_provider_calls(self, step: steps.Step):
        self.fake_lifecycle_execute.mock.assert_not_called()
        self.provider_mock.mount_project.assert_called_once_with()
        if step is None:
            self.provider_mock.execute_step.assert_not_called()
        else:
            self.provider_mock.execute_step.assert_called_once_with(step)
        self.assert_clean_not_called()

    def test_using_defaults(self):
        result = self.run_command([self.step.name])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("multipass")
        self.assert_build_provider_calls(self.step)
        self.provider_mock.shell.assert_not_called()

    def test_with_parts_specified_using_defaults(self):
        result = self.run_command([self.step.name, "part0", "part1", "part2"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("multipass")
        self.assert_build_provider_calls(self.step)
        self.provider_mock.shell.assert_not_called()

    def test_shell_using_defaults(self):
        result = self.run_command([self.step.name, "--shell"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("multipass")
        self.assert_build_provider_calls(self.previous_step)
        self.provider_mock.shell.assert_called_once_with()

    def test_shell_after_using_defaults(self):
        result = self.run_command([self.step.name, "--shell-after"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("multipass")
        self.assert_build_provider_calls(self.step)
        self.provider_mock.shell.assert_called_once_with()

    def test_using_lxd(self):
        result = self.run_command([self.step.name, "--use-lxd"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("lxd")
        self.assert_build_provider_calls(self.step)
        self.provider_mock.shell.assert_not_called()

    def test_with_parts_specified_using_lxd(self):
        result = self.run_command(
            [self.step.name, "--use-lxd", "part0", "part1", "part2"]
        )

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("lxd")
        self.assert_build_provider_calls(self.step)
        self.provider_mock.shell.assert_not_called()

    def test_using_destructive_mode(self):
        result = self.run_command([self.step.name, "--destructive-mode"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_not_called()
        self.fake_lifecycle_execute.mock.assert_called_once_with(
            self.step, mock.ANY, tuple()
        )

    def test_with_parts_specified_using_destructive_mode(self):
        result = self.run_command(
            [self.step.name, "--destructive-mode", "part0", "part1", "part2"]
        )

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_not_called()
        self.fake_lifecycle_execute.mock.assert_called_once_with(
            self.step, mock.ANY, tuple(["part0", "part1", "part2"])
        )
