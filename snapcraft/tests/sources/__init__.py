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

import unittest.mock

from snapcraft import tests


class SourceTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        patcher = unittest.mock.patch('subprocess.check_call')
        self.mock_run = patcher.start()
        self.mock_run.return_value = True
        self.addCleanup(patcher.stop)

        patcher = unittest.mock.patch('os.rmdir')
        self.mock_rmdir = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = unittest.mock.patch('os.path.exists')
        self.mock_path_exists = patcher.start()
        self.mock_path_exists.return_value = False
        self.addCleanup(patcher.stop)
