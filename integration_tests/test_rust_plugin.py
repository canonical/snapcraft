# Copyright (C) 2016 Marius Gripsgard (mariogrip@ubuntu.com)
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

import os, integration_tests

class RustPluginTestCase(integration_tests.TestCase):

    def test_stage_rust_plugin(self):
        project_dir = 'simple-rust'
        self.run_snapcraft('stage', project_dir)

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join('stage', 'bin', 'simple-rust'), cwd=project_dir)
        self.assertEqual("There is rust on snaps!\n", binary_output)
