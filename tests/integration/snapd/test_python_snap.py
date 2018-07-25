# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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

from tests import fixture_setup, integration, os_release


class PythonTestCase(integration.SnapdIntegrationTestCase):
    def test_install_and_execution(self):
        self.copy_project_to_cwd("python-hello")
        if os_release.get_version_codename() != "xenial":
            integration.add_stage_packages(
                part_name="python-part", stage_packages=["libc6"]
            )

        with fixture_setup.WithoutSnapInstalled("python-hello"):
            self.run_snapcraft()
            self.install_snap()
            self.assertThat(
                subprocess.check_output(["python-hello"], universal_newlines=True),
                Equals("Hello world!\n"),
            )
