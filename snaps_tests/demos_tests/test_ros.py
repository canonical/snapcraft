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

import os
import re
import subprocess
from unittest import skipUnless

from testtools.matchers import Equals

import snapcraft
from snapcraft.internal.common import get_os_release_info
import snaps_tests


class ROSTestCase(snaps_tests.SnapsTestCase):

    snap_content_dir = 'ros'

    @skipUnless(get_os_release_info()['VERSION_CODENAME'] == 'xenial',
                'This test fails on yakkety LP: #1614476')
    def test_ros(self):
        try:
            failed = True
            snap_path = self.build_snap(self.snap_content_dir, timeout=1800)
            failed = False
        except snaps_tests.CommandError:
            if snapcraft.ProjectOptions().deb_arch == 'arm64':
                # https://bugs.launchpad.net/snapcraft/+bug/1662915
                self.expectFailure(
                    'There are no arm64 Indigo packages in the ROS archive',
                    self.assertFalse, failed)
            else:
                raise

        self.install_snap(snap_path, 'ros-example', '1.0')
        # check that the hardcoded /usr/bin/python in rosversion
        # is changed to using /usr/bin/env python
        expected = b'#!/usr/bin/env python\n'
        output = subprocess.check_output(
            "sed -n '/env/p;1q' prime/usr/bin/rosversion",
            cwd=os.path.join(self.path, self.snap_content_dir), shell=True)
        self.assertThat(output, Equals(expected))

        # Regression test for LP: #1660852. Make sure --help actually gets
        # passed to rosaunch instead of being eaten by setup.sh.
        self.assert_command_in_snappy_testbed_with_regex([
            '/snap/bin/ros-example.launch-project', '--help'],
            r'.*Usage: roslaunch.*')

        # Run the ROS system. By default this will never exit, but the demo
        # supports an `exit-after-receive` parameter that, if true, will cause
        # the system to shutdown after the listener has successfully received
        # a message.
        self.assert_command_in_snappy_testbed_with_regex([
            '/snap/bin/ros-example.launch-project',
            'exit-after-receive:=true'], r'.*I heard Hello world.*', re.DOTALL)
