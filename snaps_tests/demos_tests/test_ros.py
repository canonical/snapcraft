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
import subprocess
from platform import linux_distribution
from unittest import skipUnless


class ROSTestCase(snaps_tests.SnapsTestCase):

    snap_content_dir = 'ros'

    @skipUnless(linux_distribution()[2] == 'xenial',
                'This test fails on yakkety LP: #1614476')
    def test_ros(self):
        snap_path = self.build_snap(self.snap_content_dir, timeout=1800)
        self.install_snap(snap_path, 'ros-example', '1.0')
        # check that the hardcoded /usr/bin/python in rosversion
        # is changed to using /usr/bin/env python
        expected = b'#!/usr/bin/env python\n'
        output = subprocess.check_output(
            "sed -n '/env/p;1q' prime/usr/bin/rosversion",
            cwd=os.path.join(self.path, self.snap_content_dir), shell=True)
        self.assertEqual(output, expected)
