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

import subprocess

from testtools.matchers import Contains

import integration_tests
from snapcraft.tests import fixture_setup


class CatkinTestCase(integration_tests.SnapdIntegrationTestCase):

    def test_catkin_pip_support(self):
        with fixture_setup.WithoutSnapInstalled('ros-pip-example'):
            self.run_snapcraft(project_dir='ros-pip')
            self.install_snap()

            # If pip support didn't work properly, the import should fail.
            self.assertThat(
                subprocess.check_output(
                    ['ros-pip-example.launch-project'],
                    universal_newlines=True, stderr=subprocess.STDOUT),
                Contains("Local timezone:"))
