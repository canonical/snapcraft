# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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
import subprocess

from testtools.matchers import (
    DirExists,
    Not
)

import integration_tests


class GulpPluginTestCase(integration_tests.TestCase):

    def test_stage_make_plugin(self):
        project_dir = 'simple-gulp'
        self.run_snapcraft('stage', project_dir)

        binary_output = subprocess.check_output(
            [os.path.join('stage', 'hello-world')],
            cwd=project_dir)
        self.assertEqual(b'I was installed with gulp\n', binary_output)
