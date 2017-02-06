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
import integration_tests


class AntPluginTestCase(integration_tests.TestCase):

    def test_build_ant_plugin(self):
        self.run_snapcraft('prime', 'ant-with-options')
        jar_path = os.path.join(self.prime_dir, 'jar', 'foo.jar')
        self.assertTrue(os.path.exists(jar_path))
