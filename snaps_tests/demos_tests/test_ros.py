
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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


class ROSTestCase(snaps_tests.SnapsTestCase):

    demo_dir = 'ros'

    def test_ros(self):
        self.build_snap(self.demo_dir)
        self.install_snap(self.demo_dir, 'ros-example', '1.0')
        # check that the hardcoded /usr/bin/python in rosversion
        # is changed to using /usr/bin/env python
        expected = b'#!/usr/bin/env python\n'
        output = subprocess.check_output(
            "sed -n '/env/p;1q' snap/usr/bin/rosversion",
            cwd=os.path.join('demos', self.demo_dir), shell=True)
        self.assertEqual(output, expected)
