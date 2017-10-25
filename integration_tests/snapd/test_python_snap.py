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

from testtools.matchers import Equals

import integration_tests
from snapcraft.tests import fixture_setup


class PythonTestCase(integration_tests.SnapdIntegrationTestCase):

    def test_install_and_execution(self):
        with fixture_setup.WithoutSnapInstalled('python-hello'):
            self.run_snapcraft(project_dir='python-hello')
            self.install_snap()
            self.assertThat(
                subprocess.check_output(
                    ['python-hello'], universal_newlines=True),
                Equals('Hello world!\n'))
