# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import integration_tests


class StagePackageVersionTestCase(integration_tests.TestCase):

    def test_stage_package_with_invalid_version_must_fail(self):
        self.copy_project_to_cwd('stage-package')
        self.set_stage_package_version(
            os.path.join('snap', 'snapcraft.yaml'),
            part='hello', package='hello', version='invalid')
        error = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft, 'pull')
        self.assertIn(
            "Error downloading stage packages for part 'hello': "
            "The package 'hello=invalid' was not found.",
            str(error.output)
        )
