# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Tim Süberkrüb
# Copyright (C) 2017-2018 Canonical Ltd
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


class SevenZipTestCase(integration.TestCase):

    _7z_test_files = {"test1.txt", "test2.txt", "test3.txt"}

    def test_stage_7z(self):
        self.run_snapcraft("stage", "7z-hello")

        for filename in self._7z_test_files:
            self.assertThat(os.path.join(self.stage_dir, filename), FileExists())
