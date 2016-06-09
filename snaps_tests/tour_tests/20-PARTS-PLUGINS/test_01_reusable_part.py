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


import snaps_tests


class ReusablePartTestCase(snaps_tests.SnapsTestCase):

    snap_content_dir = '01-reusable-part'

    def test_hello(self):
        self.build_snap(self.snap_content_dir)
        self.install_snap(self.snap_content_dir, 'hello-world-desktop', '0.1')
        self.run_command_in_snappy_testbed('ls /snap/bin/hello-world-desktop')
