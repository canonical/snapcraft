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


class BaseExampleTestCase(snaps_tests.SnapsTestCase):

    snap_content_dir = "base-example"

    def test_base_example(self):
        # Build snap will raise an exception in case of error.
        snap_path = self.build_snap(self.snap_content_dir)
        # Install snap will raise an exception in case of error.
        self.install_snap(snap_path, "base-example", "1.0")
        # more testing is done in test_base-consumer
