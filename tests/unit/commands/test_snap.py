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

import os
from typing import Optional
from unittest import mock

from testtools.matchers import Contains, Equals

from snapcraft.internal import steps

from . import LifecycleCommandsBaseTestCase


class TestSnap(LifecycleCommandsBaseTestCase):
    def assert_build_provider_calls(
        self, shell: bool = False, output: Optional[str] = None
    ):
        self.fake_lifecycle_execute.mock.assert_not_called()

        if shell:
            self.provider_mock.shell.assert_called_once_with()
        else:
            self.provider_mock.pack_project.assert_called_once_with(output=output)
            self.provider_mock.shell.assert_not_called()

        self.assert_clean_not_called()

    def test_using_defaults(self):
        result = self.run_command(["snap"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("multipass")
        self.assert_build_provider_calls()

    def test_output_using_defaults(self):
        result = self.run_command(["snap", "--output", "foo.snap"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("multipass")
        self.assert_build_provider_calls(output="foo.snap")

    def test_output_using_defaults_with_dir(self):
        result = self.run_command(["snap", "--output", "/tmp"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("multipass")
        self.assert_build_provider_calls(output="/tmp")

    def test_shell_using_defaults(self):
        result = self.run_command(["snap", "--shell"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("multipass")
        self.assert_build_provider_calls(shell=True)
        self.provider_mock.execute_step.assert_called_once_with(steps.PRIME)

    def test_shell_after_using_defaults(self):
        result = self.run_command(["snap", "--shell-after"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("multipass")
        self.assert_build_provider_calls(shell=True)
        self.provider_mock.execute_step.assert_not_called()

    def test_using_lxd(self):
        result = self.run_command(["snap", "--use-lxd"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_called_once_with("lxd")
        self.assert_build_provider_calls()

    def test_using_destructive_mode(self):
        result = self.run_command(["snap", "--destructive-mode"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_not_called()
        self.fake_lifecycle_execute.mock.assert_called_once_with(
            steps.PRIME, mock.ANY, tuple()
        )
        self.fake_pack.mock.assert_called_once_with(
            os.path.join(self.path, "prime"), compression=None, output=None
        )

    def test_output_using_destructive_mode(self):
        result = self.run_command(
            ["snap", "--destructive-mode", "--output", "foo.snap"]
        )

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_not_called()
        self.fake_lifecycle_execute.mock.assert_called_once_with(
            steps.PRIME, mock.ANY, tuple()
        )
        self.fake_pack.mock.assert_called_once_with(
            os.path.join(self.path, "prime"), compression=None, output="foo.snap"
        )

    def test_output_using_destructive_mode_with_directory(self):
        result = self.run_command(["snap", "--destructive-mode", "--output", "/tmp"])

        self.assertThat(result.exit_code, Equals(0))
        self.fake_get_provider_for.mock.assert_not_called()
        self.fake_lifecycle_execute.mock.assert_called_once_with(
            steps.PRIME, mock.ANY, tuple()
        )
        self.fake_pack.mock.assert_called_once_with(
            os.path.join(self.path, "prime"), compression=None, output="/tmp"
        )

    def test_deprecated_snap_dir(self):
        result = self.run_command(["snap", "snap-dir"])

        self.assertThat(result.exit_code, Equals(0))
        self.assertThat(result.output, Contains("DEPRECATED"))
        self.fake_get_provider_for.mock.assert_not_called()
        self.fake_lifecycle_execute.mock.assert_not_called()
        self.fake_pack.mock.assert_called_once_with("snap-dir", output=None)
