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


class BusyBoxTestCase(examples_tests.ExampleTestCase):

    example_dir = 'busybox'

    def test_busybox(self):
        self.build_snap(self.example_dir)
        self.install_snap(self.example_dir, 'busybox', '1.0')
        # Run the binary once to create the data dir.
        self.run_command_in_snappy_testbed([
            '/snap/bin/busybox.touch', '/tmp/'])
        self.assert_command_in_snappy_testbed(
            ['/snap/bin/busybox.touch',
             '~/snap/busybox/$(ls ~/snap/busybox)/busybox.test'],
            '')
        self.addCleanup(
            self.run_command_in_snappy_testbed,
            ['rm', '~/snap/busybox/*/busybox.test'])
        self.assert_command_in_snappy_testbed(
            ['/snap/bin/busybox.ls', '~/snap/busybox/*/'],
            'busybox.test\n')
