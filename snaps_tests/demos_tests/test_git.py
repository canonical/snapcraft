# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016, 2017 Canonical Ltd
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


class GitTestCase(snaps_tests.SnapsTestCase):

    snap_content_dir = 'git'

    def test_gopaste(self):
        if platform.machine() == 'armv7l':
            self.skipTest("Snaps can't yet be installed in a lxc container.")

        # Building classic snaps require the core snap to be installed
        self.install_store_snap('core')

        snap_path = self.build_snap(self.snap_content_dir)
        snap_name = 'git'
        # TODO reenable git once snap-confine and snapd bits are in place
        # self.install_snap(snap_path, snap_name, '2.8.0')
