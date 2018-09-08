# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

import fixtures
from testtools.matchers import Equals

from . import LifecycleCommandsBaseTestCase
from tests.unit.build_providers import ProviderImpl
from snapcraft.internal import steps
from snapcraft.internal.build_providers.errors import ProviderExecError


class BuildProviderDebugCommandTestCase(LifecycleCommandsBaseTestCase):
    def setUp(self):
        super().setUp()
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT", "multipass")
        )

        shell_mock = mock.Mock()

        class Provider(ProviderImpl):
            def execute_step(self, step: steps.Step) -> None:
                raise ProviderExecError(
                    provider_name="fake", command=["snapcraft", "pull"], exit_code=1
                )

            def shell(self):
                shell_mock()

        patcher = mock.patch(
            "snapcraft.internal.build_providers.get_provider_for", return_value=Provider
        )
        self.provider = patcher.start()
        self.addCleanup(patcher.stop)

        self.shell_mock = shell_mock

        self.make_snapcraft_yaml("pull", base="core18")

    def test_step_with_debug_using_build_provider_fails(self):
        result = self.run_command(["--debug", "pull"])

        self.assertThat(result.exit_code, Equals(0))
        self.shell_mock.assert_called_once_with()

    def test_step_with_debug_after_pull_using_build_provider_fails(self):
        result = self.run_command(["pull", "--debug"])

        self.assertThat(result.exit_code, Equals(0))
        self.shell_mock.assert_called_once_with()

    def test_step_without_debug_using_build_provider_fails_and_does_not_shell(self):
        self.assertRaises(ProviderExecError, self.run_command, ["pull"])

        self.shell_mock.assert_not_called()
