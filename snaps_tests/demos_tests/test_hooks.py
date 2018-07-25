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

import subprocess
import snaps_tests


class HookCase(snaps_tests.SnapsTestCase):

    snap_content_dir = "hooks"

    def test_hooks(self):
        snap_path = self.build_snap(self.snap_content_dir)
        self.install_snap(snap_path, "hooks", "1.0")

        # Regular `snap set` should succeed.
        self.run_command_in_snappy_testbed("sudo snap set hooks foo=bar")

        if not snaps_tests.config.get("skip-install", False):
            # Setting fail=true should fail.
            self.assertRaises(
                subprocess.CalledProcessError,
                self.run_command_in_snappy_testbed,
                "sudo snap set hooks fail=true",
            )
