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
from distutils import dir_util

from testtools.matchers import Equals

import integration_tests
from snapcraft.tests import fixture_setup


class GoTestCase(integration_tests.SnapdIntegrationTestCase):

    def test_install_and_execution(self):
        with fixture_setup.WithoutSnapInstalled('go-hello'):
            # If we just put the source in the temp dir, go will generate
            # the binary with the name of the temp dir, instead of go-hello.
            dir_util.copy_tree(
                os.path.join(self.snaps_dir, 'go-hello'),
                os.path.join(self.path, 'go-hello'),
                preserve_symlinks=True)
            os.chdir('go-hello')

            self.run_snapcraft()
            self.install_snap()
            self.assertThat(
                subprocess.check_output(
                    ['go-hello'], universal_newlines=True),
                Equals('Hello snapcrafter\n'))
