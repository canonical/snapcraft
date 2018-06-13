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
import shutil
import xdg

from testtools.matchers import Contains, Equals, Not

from tests import integration


class CmakePluginTestCase(integration.TestCase):

    def test_stage_cmake_plugin(self):
        self.run_snapcraft('stage', 'cmake-hello')

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.stage_dir, 'bin', 'cmake-hello'))
        self.assertThat(binary_output, Equals("It's a CMake world\n"))

    def test_cmake_can_rebuild(self):
        # Flip the config switch to automatically update
        config_path = os.path.join(
            xdg.BaseDirectory.save_config_path('snapcraft'), 'cli.cfg')
        with open(config_path, 'w') as f:
            f.write('[Lifecycle]\noutdated_step_action = clean')

        self.copy_project_to_cwd('cmake-with-lib')
        output = self.run_snapcraft('prime')

        # Assert that cmake actually configured and built from scratch
        self.assertThat(output, Contains('The CXX compiler identification'))
        self.assertThat(
            output,
            Contains('Building CXX object CMakeFiles/foo.dir/foo.cpp.o'))
        self.assertThat(
            output,
            Contains('Building CXX object CMakeFiles/usefoo.dir/main.cpp.o'))

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.prime_dir, 'bin', 'usefoo'))
        self.assertThat(binary_output, Equals("foo\n"))

        # Modify the source code
        source_file_path = os.path.join('src', 'foo.cpp')
        shutil.copy('new_foo.cpp', source_file_path)

        # Prime again. This should rebuild
        output = self.run_snapcraft('prime')

        # Assert that cmake did not start from scratch, and reused everything
        # it could
        self.assertThat(
            output, Not(Contains('The CXX compiler identification')))
        self.assertThat(
            output,
            Contains('Building CXX object CMakeFiles/foo.dir/foo.cpp.o'))
        self.assertThat(
            output,
            Not(Contains(
                'Building CXX object CMakeFiles/usefoo.dir/main.cpp.o')))

        binary_output = self.get_output_ignoring_non_zero_exit(
            os.path.join(self.prime_dir, 'bin', 'usefoo'))
        self.assertThat(binary_output, Equals("new foo\n"))
