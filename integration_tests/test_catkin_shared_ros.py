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
import tempfile

import integration_tests


class CatkinSharedRosTestCase(integration_tests.TestCase):

    def test_shared_ros_builds_without_catkin_in_underlay(self):
        # Build the producer until we have a good staging area
        self.copy_project_to_cwd(os.path.join('catkin-shared-ros', 'producer'))
        self.run_snapcraft('stage')

        with tempfile.TemporaryDirectory() as tmpdir:
            underlay_tarball = os.path.join(tmpdir, 'underlay.tar.bz2')

            # Now tar up the producer's staging area to be used in the consumer
            subprocess.check_call(['tar', 'czf', underlay_tarball, 'stage/'])

            # Blow away the entire producer project
            self.run_snapcraft('clean')
            subprocess.check_call(['rm', '-rf', '*'])

            # Copy the tarball back into cwd
            os.rename(underlay_tarball, 'underlay.tar.bz2')

        # Now copy in and build the consumer. This should not throw exceptions.
        self.copy_project_to_cwd(os.path.join('catkin-shared-ros', 'consumer'))
        self.run_snapcraft('build')
