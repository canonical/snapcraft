# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Tim Süberkrüb
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

import integration_tests


class SevenZipTestCase(integration_tests.TestCase):

    def test_stage_7z(self):
        self.run_snapcraft('stage', '7z-hello')

        self.assertThat(
            os.path.join(self.stage_dir, 'test.txt'),
            FileExists()
        )
