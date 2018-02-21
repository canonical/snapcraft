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

from snapcraft.tests import (
    integration,
    fixture_setup,
    skip,
)


class AmentTestCase(integration.SnapdIntegrationTestCase):

    slow_test = True

    @skip.skip_unless_codename(
        'xenial', 'Anything later than xenial will die with NO_PUBKEY')
    def test_ament_support(self):
        self.useFixture(fixture_setup.WithoutSnapInstalled('ros2-example'))
        self.run_snapcraft(project_dir='ros2')
        self.install_snap()

        self.assertThat(
            subprocess.check_output(
                ['ros2-example.launch-project'],
                universal_newlines=True, stderr=subprocess.STDOUT),
            Contains('I heard: Hello world'))
