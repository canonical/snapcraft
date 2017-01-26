# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016, 2017 Canonical Ltd
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

import integration_tests
import os
from testtools.matchers import FileExists


class NodeJSPluginTestCase(integration_tests.TestCase):

    def test_rebuilding_possible(self):
        self.copy_project_to_cwd('simple-nodejs')
        self.run_snapcraft('build')
        self.run_snapcraft(['clean', '-s', 'build'])
        self.run_snapcraft('build')

    def test_build_with_run_commands(self):
        self.run_snapcraft('build', 'nodejs-with-run-commands')
        self.assertThat(
            os.path.join(self.parts_dir, 'nodejs-with-run', 'build',
                         'command-one-run'),
            FileExists())
        self.assertThat(
            os.path.join(self.parts_dir, 'nodejs-with-run', 'build',
                         'command-two-run'),
            FileExists())
