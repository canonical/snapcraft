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

import snaps_tests


class GoddTestCase(snaps_tests.SnapsTestCase):

    snap_content_dir = 'godd'

    def test_godd(self):
        # Build snap will raise an exception in case of error.
        snap_path = self.build_snap(self.snap_content_dir)
        # Install snap will raise an exception in case of error.
        self.install_snap(snap_path, 'godd', '1.0')

        self.run_command_in_snappy_testbed(
            'sudo snap connect godd:mount-observe')
        self.run_command_in_snappy_testbed('mkdir -p ~/snap/godd/common')
        self.run_command_in_snappy_testbed('touch ~/snap/godd/common/test')
        self.run_command_in_snappy_testbed(
            'godd ~/snap/godd/common/test /dev/null')
