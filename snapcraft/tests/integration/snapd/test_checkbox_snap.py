# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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

from snapcraft.tests import (
    fixture_setup,
    integration
)


class PlainboxTestCase(integration.SnapdIntegrationTestCase):

    def test_install_and_execution(self):
        with fixture_setup.WithoutSnapInstalled('plainbox-simple'):
            self.run_snapcraft(project_dir='plainbox-simple')
            self.install_snap()

            # This test fails if the binary is executed from /tmp.
            os.chdir(os.path.expanduser('~'))

            expected = textwrap.dedent("""\
                com.canonical.plainbox::collect-manifest
                com.canonical.plainbox::manifest
                com.example::always-fail
                com.example::always-pass
                """)
            self.assertThat(
                subprocess.check_output(
                    ['plainbox-simple.plainbox', 'dev', 'special', '-j'],
                    universal_newlines=True),
                Equals(expected))

            subprocess.check_call(
                ['plainbox-simple.plainbox', 'run',
                 '-i com.example::.*'])
