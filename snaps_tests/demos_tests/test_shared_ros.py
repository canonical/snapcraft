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


class SharedROSTestCase(snaps_tests.SnapsTestCase):

    snap_content_dir = 'shared-ros'

    @skipUnless(linux_distribution()[2] == 'xenial',
                'This test fails on yakkety LP: #1614476')
    def test_shared_ros(self):
        ros_base_path = os.path.join(self.snap_content_dir, 'ros-base')
        ros_app_path = os.path.join(self.snap_content_dir, 'ros-app')

        base_snap_path = self.build_snap(ros_base_path, timeout=1800)

        # Now tar up its staging area to be used to build ros-app
        subprocess.check_call([
            'tar', 'czf', os.path.join(ros_app_path, 'ros-base.tar.bz2'), '-C',
            os.path.dirname(base_snap_path), 'stage'], cwd=self.src_dir)

        # Now build ros-app
        app_snap_path = self.build_snap(ros_app_path, timeout=1800)

        # Install both snaps
        self.install_snap(base_snap_path, 'ros-base', '1.0')
        self.install_snap(app_snap_path, 'ros-app', '1.0')

        # Connect the content sharing interface
        self.run_command_in_snappy_testbed(
            'sudo snap connect ros-app:ros-base ros-base:ros-base')

        # Make sure the talker/listener system actually comes up by verifying
        # that the listener receives something after 5 seconds. `timeout` has
        # a `--preserve-status` option, but it doesn't always work, so we'll
        # leave it off and just catch the subprocess error.
        try:
            self.snappy_testbed.run_command(
                ['timeout', '5s', '/snap/bin/ros-app.launch-project'])
        except subprocess.CalledProcessError as e:
            self.assertThat(e.output.decode('utf8'), MatchesRegex(
                r'.*I heard Hello world.*', re.DOTALL))
