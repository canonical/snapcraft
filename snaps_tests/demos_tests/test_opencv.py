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
from testtools.matchers import ContainsAll

import snaps_tests


class OpenCVTestCase(snaps_tests.SnapsTestCase):

    snap_content_dir = 'opencv'

    def test_opencv(self):
        snap_path = self.build_snap(self.snap_content_dir)
        self.install_snap(snap_path, 'opencv-example', '1.0')
        if not snaps_tests.config.get('skip-install', False):
            output = self.run_command_in_snappy_testbed(
                '/snap/bin/opencv-example.example').splitlines()

            # Depending on opencv the result is now displayed differently
            # so let's do a lazy match.
            # On artful you see:
            # [  1,   3;
            #    2,   4]
            # And on others:
            # [1, 3;
            #  2, 4]
            expected_in_first_line = ['[', '1', ',', '3', ';']
            self.assertThat(output[0], ContainsAll(expected_in_first_line))
            expected_in_second_line = ['2', ',', '4', ']']
            self.assertThat(output[1], ContainsAll(expected_in_second_line))
