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

import os
import subprocess
import tempfile

from testtools.matchers import FileContains, FileExists

from tests import integration, skip


class CatkinTestCase(integration.TestCase):

    @skip.skip_unless_codename('xenial', 'ROS Kinetic only targets Xenial')
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

    @skip.skip_unless_codename('xenial', 'ROS Kinetic only targets Xenial')
    def test_catkin_part_builds_after_python_part(self):
        self.copy_project_to_cwd('catkin-with-python-part')

        # This snap should be staged with no errors
        self.run_snapcraft('stage')

    @skip.skip_unless_codename('xenial', 'ROS Kinetic only targets Xenial')
    def test_catkin_recursively_parses_rosinstall_files(self):
        self.copy_project_to_cwd('catkin-recursive-rosinstall')

        # Unpack the git repos for the test
        subprocess.check_call(['tar', 'xf', 'repos.tar'])

        # Pull should have no errors
        self.run_snapcraft('pull')

        repo1_path = os.path.join(
            self.parts_dir, 'catkin-part', 'src', 'src', 'repo1')
        repo2_path = os.path.join(
            self.parts_dir, 'catkin-part', 'src', 'src', 'repo2')
        repo3_path = os.path.join(
            self.parts_dir, 'catkin-part', 'src', 'src', 'repo3')

        self.assertThat(
            os.path.join(repo1_path, 'repo1.rosinstall'), FileExists())
        self.assertThat(os.path.join(repo1_path, 'file1'), FileContains('1\n'))

        self.assertThat(
            os.path.join(repo2_path, 'repo2.rosinstall'), FileExists())
        self.assertThat(os.path.join(repo2_path, 'file2'), FileContains('2\n'))

        self.assertThat(os.path.join(repo3_path, 'file3'), FileContains('3\n'))
