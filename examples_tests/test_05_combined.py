# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

import examples_tests


class HelloWorldDesktopTestCase(examples_tests.ExampleTestCase):

    snap_content_dir = '05-hello-world-combined'

    def test_hello_world_service(self):
        self.build_snap(self.snap_content_dir)
        self.install_snap(self.snap_content_dir, 'hello-world-combined', '0.1')
        self.assert_command_in_snappy_testbed(
            '/snap/bin/hello-world-combined.hello-world-cli', 'Hello World\n')
        snap_name = 'hello-world-service'
        self.assert_service_running(snap_name, 'hello-service')
        self.run_command_in_snappy_testbed('ls /snap/bin/hello-world-combined.hello-world-desktop')
