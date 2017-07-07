# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (c) 2016 James Beedy <jamesbeedy@gmail.com>
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

import snaps_tests


class RubyHelloTestCase(snaps_tests.SnapsTestCase):

    snap_content_dir = "ruby-hello"

    def test_ruby(self):
        # Build snap will raise an exception in case of error.
        snap_path = self.build_snap(self.snap_content_dir)
        # Install snap will raise an exception in case of error.
        self.install_snap(snap_path, "ruby-hello", "1.0")

        self.assert_command_in_snappy_testbed_with_regex([
            '/snap/bin/ruby-hello'], r'.*Ruby says, Hello Snapcraft!.*',
                re.DOTALL)
