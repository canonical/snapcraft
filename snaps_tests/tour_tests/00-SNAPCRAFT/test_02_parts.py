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


class EasyStartTestCase(snaps_tests.SnapsTestCase):

    snap_content_dir = '02-parts'

    def test_hello(self):
        self.skipTest('Skip while the proxy rule is added by IS.')
        self.build_snap(self.snap_content_dir)
        self.install_snap(self.snap_content_dir, 'hello-debug', '2.10')
        self.run_command_in_snappy_testbed(
            '/snap/bin/hello-debug.bash --version')
