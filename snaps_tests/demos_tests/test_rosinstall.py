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

import re

from snaps_tests import SnapsTestCase, skip


class RosinstallTestCase(SnapsTestCase):

    snap_content_dir = 'rosinstall'

    @skip.skip_unless_codename('xenial', 'ROS Kinetic only targets Xenial')
    def test_rosinstall(self):
        snap_path = self.build_snap(self.snap_content_dir, timeout=10000)

        self.install_snap(snap_path, 'rosinstall-demo', '1.0')

        # Run the ROS system. By default this will never exit, but the demo
        # supports an `exit-after-receive` parameter that, if true, will cause
        # the system to shutdown after the listener has successfully received
        # a message.
        self.assert_command_in_snappy_testbed_with_regex([
            '/snap/bin/rosinstall-demo.run',
            'exit-after-receive:=true'], r'.*I heard hello world.*', re.DOTALL)
