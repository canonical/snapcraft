# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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
import re
import subprocess

from testtools.matchers import (
    Contains,
    Equals,
    MatchesRegex,
)

from snapcraft.tests import (
    integration,
    fixture_setup,
    skip,
)


class CatkinTestCase(integration.SnapdIntegrationTestCase):

    slow_test = True

    @skip.skip_unless_codename('xenial', 'ROS Kinetic only targets Xenial')
    def test_install_and_execution(self):
        self.useFixture(fixture_setup.WithoutSnapInstalled('ros-example'))
        try:
            failed = True
            self.run_snapcraft(project_dir='ros-talker-listener')
            failed = False
        except subprocess.CalledProcessError:
            if self.deb_arch == 'arm64':
                # https://bugs.launchpad.net/snapcraft/+bug/1662915
                self.expectFailure(
                    'There are no arm64 Indigo packages in the ROS archive',
                    self.assertFalse, failed)
            else:
                raise

        self.install_snap()
        # check that the hardcoded /usr/bin/python in rosversion
        # is changed to using /usr/bin/env python
        expected = b'#!/usr/bin/env python\n'
        output = subprocess.check_output(
            "sed -n '/env/p;1q' prime/usr/bin/rosversion", shell=True)
        self.assertThat(output, Equals(expected))

        # This test fails if the binary is executed from /tmp.
        os.chdir(os.path.expanduser('~'))
        # Regression test for LP: #1660852. Make sure --help actually gets
        # passed to roslaunch instead of being eaten by setup.sh.
        output = subprocess.check_output(
            ['ros-example.launch-project', '--help']).decode()
        self.assertThat(output, MatchesRegex(r'.*Usage: roslaunch.*'))

        # Run the ROS system. By default this will never exit, but the demo
        # supports an `exit-after-receive` parameter that, if true, will cause
        # the system to shutdown after the listener has successfully received
        # a message.
        output = subprocess.check_output(
            ['ros-example.launch-project',
             'exit-after-receive:=true']).decode()
        self.assertThat(
            output,
            MatchesRegex(r'.*I heard Hello world.*', flags=re.DOTALL))

    @skip.skip_unless_codename('xenial', 'ROS Kinetic only targets Xenial')
    def test_catkin_pip_support(self):
        with fixture_setup.WithoutSnapInstalled('ros-pip-example'):
            self.run_snapcraft(project_dir='ros-pip')
            self.install_snap()

            # If pip support didn't work properly, the import should fail.
            self.assertThat(
                subprocess.check_output(
                    ['ros-pip-example.launch-project'],
                    universal_newlines=True, stderr=subprocess.STDOUT),
                Contains("Local timezone:"))
