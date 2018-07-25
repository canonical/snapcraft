# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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

from testtools.matchers import FileExists

from tests import integration


class DownloadTestCase(integration.StoreTestCase):
    def setUp(self):
        if os.getenv("TEST_STORE") == "staging":
            # TODO add the snap to the staging server.
            self.skipTest("There is no core snap in the staging server")
        super().setUp()

    def test_download_os_snap(self):
        self.run_snapcraft("pull", "kernel-download")
        self.assertThat(
            os.path.join(self.parts_dir, "kernel", "src", "os.snap"), FileExists()
        )
