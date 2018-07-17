# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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
import textwrap

from testtools.matchers import Equals

from tests import fixture_setup, integration


class CheckboxTestCase(integration.SnapdIntegrationTestCase):
    def test_install_and_execution(self):
        with fixture_setup.WithoutSnapInstalled("checkbox-simple"):
            self.run_snapcraft(project_dir="checkbox-simple")
            self.install_snap()

            # This test fails if the binary is executed from /tmp.
            os.chdir(os.path.expanduser("~"))

            expected = textwrap.dedent(
                """\
                job 'com.canonical.plainbox::collect-manifest'
                job 'com.canonical.plainbox::manifest'
                job 'com.example::always-pass'
                job 'com.example::always-fail'
                """
            )
            self.assertThat(
                subprocess.check_output(
                    ["checkbox-simple.checkbox-cli", "list", "job"],
                    universal_newlines=True,
                ),
                Equals(expected),
            )

            subprocess.check_call(
                ["checkbox-simple.checkbox-cli", "run", "com.example::always-pass"]
            )
