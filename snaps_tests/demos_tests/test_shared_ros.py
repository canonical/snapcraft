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

from snaps_tests import SnapsTestCase, skip


class SharedROSTestCase(SnapsTestCase):

    snap_content_dir = "shared-ros"

    @skip.skip_unless_codename("xenial", "ROS Kinetic only targets Xenial")
    def test_shared_ros(self):
        ros_base_path = os.path.join(self.snap_content_dir, "ros-base")
        ros_app_path = os.path.join(self.snap_content_dir, "ros-app")

        base_snap_path = self.build_snap(ros_base_path, timeout=10000)

        # Now tar up its staging area to be used to build ros-app
        subprocess.check_call(
            [
                "tar",
                "czf",
                os.path.join(ros_app_path, "ros-base.tar.bz2"),
                "-C",
                os.path.dirname(base_snap_path),
                "stage",
            ],
            cwd=self.src_dir,
        )

        # Now build ros-app
        app_snap_path = self.build_snap(ros_app_path, timeout=10000)

        # Install both snaps
        self.install_snap(base_snap_path, "ros-base", "1.0")
        self.install_snap(app_snap_path, "ros-app", "1.0")

        # Connect the content sharing interface
        self.run_command_in_snappy_testbed(
            "sudo snap connect ros-app:ros-base ros-base:ros-base"
        )

        # Run the ROS system. By default this will never exit, but the demo
        # supports an `exit-after-receive` parameter that, if true, will cause
        # the system to shutdown after the listener has successfully received
        # a message.
        self.assert_command_in_snappy_testbed_with_regex(
            ["/snap/bin/ros-app.launch-project", "exit-after-receive:=true"],
            r".*I heard Hello world.*",
            re.DOTALL,
        )
