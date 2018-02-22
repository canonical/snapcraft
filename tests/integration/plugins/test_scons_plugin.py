# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2017-2018 Canonical Ltd
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

from testtools.matchers import Equals

from tests import integration


class SconsPluginTestCase(integration.TestCase):

    def test_stage_scons_plugin(self):
        self.run_snapcraft('stage', 'scons-hello')

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.stage_dir, 'opt', 'bin', 'main'))
        self.assertThat(binary_output, Equals('Hello world\n'))
