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

from tests import fixture_setup, integration


class NodeJSTestCase(integration.SnapdIntegrationTestCase):
    def test_install_and_execution(self):
        with fixture_setup.WithoutSnapInstalled("nodejs-hello"):
            self.run_snapcraft(project_dir="nodejs-hello")
            self.install_snap()
            self.assertThat(
                subprocess.check_output(["nodejs-hello"], universal_newlines=True),
                Equals("Hello world!\n"),
            )
