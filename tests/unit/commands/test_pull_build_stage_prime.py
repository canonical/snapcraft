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
    def assert_build_provider_calls(self, step: steps.Step):
        self.fake_lifecycle_execute.mock.assert_not_called()
        if step is None:
            self.provider_mock.execute_step.assert_not_called()
        else:
            self.provider_mock.execute_step.assert_called_once_with(step)
        self.assert_clean_not_called()

    def run_test_using_defaults(self, step):
        result = self.run_command([step.name])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("multipass")
        self.assert_build_provider_calls(step)
        self.provider_mock.shell.assert_not_called()

    def run_test_with_parts_specified_using_defaults(self, step):
        result = self.run_command([step.name, "part0", "part1", "part2"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("multipass")
        self.assert_build_provider_calls(step)
        self.provider_mock.shell.assert_not_called()

    def run_test_shell_using_defaults(self, step, previous_step):
        result = self.run_command([step.name, "--shell"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("multipass")
        self.assert_build_provider_calls(previous_step)
        self.provider_mock.shell.assert_called_once_with()

    def run_test_shell_after_using_defaults(self, step):
        result = self.run_command([step.name, "--shell-after"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("multipass")
        self.assert_build_provider_calls(step)
        self.provider_mock.shell.assert_called_once_with()

    def run_test_using_lxd(self, step):
        result = self.run_command([step.name, "--use-lxd"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("lxd")
        self.assert_build_provider_calls(step)
        self.provider_mock.shell.assert_not_called()

    def run_test_with_parts_specified_using_lxd(self, step):
        result = self.run_command([step.name, "--use-lxd", "part0", "part1", "part2"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("lxd")
        self.assert_build_provider_calls(step)
        self.provider_mock.shell.assert_not_called()

    def run_test_using_destructive_mode(self, step):
        result = self.run_command([step.name, "--destructive-mode"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_not_called()
        self.fake_lifecycle_execute.mock.assert_called_once_with(
            step, mock.ANY, tuple()
        )

    def run_test_with_parts_specified_using_destructive_mode(self, step):
        result = self.run_command(
            [step.name, "--destructive-mode", "part0", "part1", "part2"]
        )

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_not_called()
        self.fake_lifecycle_execute.mock.assert_called_once_with(
            step, mock.ANY, tuple(["part0", "part1", "part2"])
        )

    def test_pull_defaults(self):
        self.run_test_using_defaults(step=steps.PULL)

    def test_pull_with_parts_specified_using_defaults(self):
        self.run_test_with_parts_specified_using_defaults(step=steps.PULL)

    def test_pull_shell_using_defaults(self):
        self.run_test_shell_using_defaults(step=steps.PULL, previous_step=None)

    def test_pull_shell_after_using_defaults(self):
        self.run_test_shell_after_using_defaults(step=steps.PULL)

    def test_pull_using_lxd(self):
        self.run_test_using_lxd(step=steps.PULL)

    def test_pull_with_parts_specified_using_lxd(self):
        self.run_test_with_parts_specified_using_lxd(step=steps.PULL)

    def test_pull_using_destructive_mode(self):
        self.run_test_using_destructive_mode(step=steps.PULL)

    def test_pull_with_parts_specified_using_destructive_mode(self):
        self.run_test_with_parts_specified_using_destructive_mode(step=steps.PULL)

    def test_build_defaults(self):
        self.run_test_using_defaults(step=steps.BUILD)

    def test_build_with_parts_specified_using_defaults(self):
        self.run_test_with_parts_specified_using_defaults(step=steps.BUILD)

    def test_build_shell_using_defaults(self):
        self.run_test_shell_using_defaults(step=steps.BUILD, previous_step=steps.PULL)

    def test_build_shell_after_using_defaults(self):
        self.run_test_shell_after_using_defaults(step=steps.BUILD)

    def test_build_using_lxd(self):
        self.run_test_using_lxd(step=steps.BUILD)

    def test_build_with_parts_specified_using_lxd(self):
        self.run_test_with_parts_specified_using_lxd(step=steps.BUILD)

    def test_build_using_destructive_mode(self):
        self.run_test_using_destructive_mode(step=steps.BUILD)

    def test_build_with_parts_specified_using_destructive_mode(self):
        self.run_test_with_parts_specified_using_destructive_mode(step=steps.BUILD)

    def test_stage_defaults(self):
        self.run_test_using_defaults(step=steps.STAGE)

    def test_stage_with_parts_specified_using_defaults(self):
        self.run_test_with_parts_specified_using_defaults(step=steps.STAGE)

    def test_stage_shell_using_defaults(self):
        self.run_test_shell_using_defaults(step=steps.STAGE, previous_step=steps.BUILD)

    def test_stage_shell_after_using_defaults(self):
        self.run_test_shell_after_using_defaults(step=steps.STAGE)

    def test_stage_using_lxd(self):
        self.run_test_using_lxd(step=steps.STAGE)

    def test_stage_with_parts_specified_using_lxd(self):
        self.run_test_with_parts_specified_using_lxd(step=steps.STAGE)

    def test_stage_using_destructive_mode(self):
        self.run_test_using_destructive_mode(step=steps.STAGE)

    def test_stage_with_parts_specified_using_destructive_mode(self):
        self.run_test_with_parts_specified_using_destructive_mode(step=steps.STAGE)

    def test_prime_defaults(self):
        self.run_test_using_defaults(step=steps.PRIME)

    def test_prime_with_parts_specified_using_defaults(self):
        self.run_test_with_parts_specified_using_defaults(step=steps.PRIME)

    def test_prime_shell_using_defaults(self):
        self.run_test_shell_using_defaults(step=steps.PRIME, previous_step=steps.STAGE)

    def test_prime_shell_after_using_defaults(self):
        self.run_test_shell_after_using_defaults(step=steps.PRIME)

    def test_prime_using_lxd(self):
        self.run_test_using_lxd(step=steps.PRIME)

    def test_prime_with_parts_specified_using_lxd(self):
        self.run_test_with_parts_specified_using_lxd(step=steps.PRIME)

    def test_prime_using_destructive_mode(self):
        self.run_test_using_destructive_mode(step=steps.PRIME)

    def test_prime_with_parts_specified_using_destructive_mode(self):
        self.run_test_with_parts_specified_using_destructive_mode(step=steps.PRIME)
