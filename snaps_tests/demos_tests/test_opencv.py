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
import os
import subprocess

from testtools.matchers import Contains, ContainsAll, FileExists, MatchesRegex

import snapcraft
import snaps_tests
from snaps_tests import skip


class OpenCVTestCase(snaps_tests.SnapsTestCase):

    snap_content_dir = "opencv"

    @skip.skip_unless_codename("xenial", "declared stage-packages only in xenial")
    def test_opencv(self):
        if snapcraft.ProjectOptions().deb_arch == "armhf":
            self.skipTest("The autopkgtest armhf runners can't install snaps")

        snap_path = self.build_snap(self.snap_content_dir)

        bin_path = os.path.join(os.path.dirname(snap_path), "prime", "bin", "example")
        self.assertThat(bin_path, FileExists())

        interpreter = subprocess.check_output(
            [self.patchelf_command, "--print-interpreter", bin_path]
        ).decode()
        expected_interpreter = r"^/snap/.*"
        self.assertThat(interpreter, MatchesRegex(expected_interpreter))

        arch_triplet = snapcraft.ProjectOptions().arch_triplet

        # test $ORIGIN in action
        rpath = subprocess.check_output(
            [self.patchelf_command, "--print-rpath", bin_path]
        ).decode()
        expected_rpath = "$ORIGIN/../usr/lib/{}:".format(arch_triplet)
        self.assertThat(rpath, Contains(expected_rpath))

        # test $ORIGIN applied
        ldd = subprocess.check_output(["ldd", bin_path]).decode()
        expected_opencv_path = "/prime/bin/../usr/lib/{}/libopencv_core".format(
            arch_triplet
        )
        self.assertThat(ldd, Contains(expected_opencv_path))

        self.install_snap(snap_path, "opencv-example", "1.0", classic=True)
        if not snaps_tests.config.get("skip-install", False):
            output = self.run_command_in_snappy_testbed(
                "/snap/bin/opencv-example.example"
            ).splitlines()

            # Depending on opencv the result is now displayed differently
            # so let's do a lazy match.
            # On artful you see:
            # [  1,   3;
            #    2,   4]
            # And on others:
            # [1, 3;
            #  2, 4]
            expected_in_first_line = ["[", "1", ",", "3", ";"]
            self.assertThat(output[0], ContainsAll(expected_in_first_line))
            expected_in_second_line = ["2", ",", "4", "]"]
            self.assertThat(output[1], ContainsAll(expected_in_second_line))
