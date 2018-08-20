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

import re
import subprocess

from testtools.matchers import MatchesRegex

from tests import fixture_setup, integration, skip


class CatkinToolsTestCase(integration.SnapdIntegrationTestCase):

    slow_test = True

    @skip.skip_unless_codename("xenial", "ROS Kinetic only targets Xenial")
    def test_install_and_execution(self) -> None:
        self.useFixture(fixture_setup.WithoutSnapInstalled("catkin-tools-example"))
        self.run_snapcraft(project_dir="catkin-tools-talker-listener")
        self.install_snap()

        # Run the ROS system. By default this will never exit, but the code
        # supports an `exit-after-receive` parameter that, if true, will cause
        # the system to shutdown after the listener has successfully received
        # a message.
        output = subprocess.check_output(
            ["catkin-tools-example.launch-project", "exit-after-receive:=true"]
        ).decode()
        self.assertThat(
            output, MatchesRegex(r".*I heard Hello world.*", flags=re.DOTALL)
        )
