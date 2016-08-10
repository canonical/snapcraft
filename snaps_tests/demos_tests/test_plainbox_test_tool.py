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


class PlainboxTestCase(snaps_tests.SnapsTestCase):

    snap_content_dir = 'plainbox-test-tool'

    def test_plainbox(self):
        # build and install the example
        snap_path = self.build_snap(self.snap_content_dir)
        self.install_snap(snap_path, 'plainbox-test-tool', '0.1')
        # check that can run plainbox and list the tests available
        expected = """2013.com.canonical.plainbox::collect-manifest
2013.com.canonical.plainbox::manifest
2016.com.example::always-fail
2016.com.example::always-pass
"""
        self.assert_command_in_snappy_testbed(
            '/snap/bin/plainbox-test-tool.plainbox dev special -j',
            expected)
        # check can run the tests in the example provider
        self.run_command_in_snappy_testbed('/snap/bin/plainbox-test-tool.'
                                           'plainbox run '
                                           '-i 2016.com.example::.*')
