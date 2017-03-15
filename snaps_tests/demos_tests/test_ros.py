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

import snaps_tests

import os
import re
import subprocess
from platform import linux_distribution
from unittest import skipUnless
from testtools.matchers import MatchesRegex


class ROSTestCase(snaps_tests.SnapsTestCase):

    snap_content_dir = 'ros'

    @skipUnless(linux_distribution()[2] == 'xenial',
                'This test fails on yakkety LP: #1614476')
    def test_ros(self):
        snap_path = self.build_snap(self.snap_content_dir, timeout=1800)
        self.install_snap(snap_path, 'ros-example', '1.0')
        # check that the hardcoded /usr/bin/python in rosversion
        # is changed to using /usr/bin/env python
        expected = b'#!/usr/bin/env python\n'
        output = subprocess.check_output(
            "sed -n '/env/p;1q' prime/usr/bin/rosversion",
            cwd=os.path.join(self.path, self.snap_content_dir), shell=True)
        self.assertEqual(output, expected)

        # Regression test for LP: #1660852. Make sure --help actually gets
        # passed to rosaunch instead of being eaten by setup.sh.
        self.assert_command_in_snappy_testbed_with_regex([
            '/snap/bin/ros-example.launch-project', '--help'],
            '.*Usage: roslaunch.*')

        # Make sure the talker/listener system actually comes up by verifying
        # that the listener receives something after 5 seconds. `timeout` has
        # a `--preserve-status` option, but it doesn't always work, so we'll
        # leave it off and just catch the subprocess error.
        try:
            self.snappy_testbed.run_command(
                ['timeout', '5s', '/snap/bin/ros-example.launch-project'])
        except subprocess.CalledProcessError as e:
            self.assertThat(e.output.decode('utf8'), MatchesRegex(
                r'.*I heard Hello world.*', re.DOTALL))
