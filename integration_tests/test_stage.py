# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2017 Canonical Ltd
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

import fixtures
from testtools.matchers import (
    Contains,
    Equals,
    FileExists,
    Not
)

import integration_tests


class StageTestCase(integration_tests.TestCase):

    def _stage_conflicts(self, debug):
        exception = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft, 'stage', 'conflicts', debug=debug)

        self.assertThat(exception.returncode, Equals(2))
        expected_conflicts = (
            "Parts 'p1' and 'p2' have the following file paths in common "
            "which have different contents:\n    bin/test\n")
        self.assertThat(exception.output, Contains(expected_conflicts))

        expected_help = (
            'Snapcraft offers some capabilities to solve this by use '
            'of the following keywords:\n'
            '    - `filesets`\n'
            '    - `stage`\n'
            '    - `snap`\n'
            '    - `organize`\n\n'
            'Learn more about these part keywords by running '
            '`snapcraft help plugins`'
        )
        self.assertThat(exception.output, Contains(expected_help))
        return exception.output

    def test_conflicts_no_traceback_without_debug(self):
        self.assertThat(
            self._stage_conflicts(False), Not(Contains("Traceback")))

    def test_conflicts_traceback_with_debug(self):
        self.assertThat(
            self._stage_conflicts(True), Contains("Traceback"))

    def test_conflicts(self):
        exception = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft, 'stage', 'conflicts')

        self.assertThat(exception.returncode, Equals(2))
        expected_conflicts = (
            "Parts 'p1' and 'p2' have the following file paths in common "
            "which have different contents:\n    bin/test\n")
        self.assertThat(exception.output, Contains(expected_conflicts))

        expected_help = (
            'Snapcraft offers some capabilities to solve this by use '
            'of the following keywords:\n'
            '    - `filesets`\n'
            '    - `stage`\n'
            '    - `snap`\n'
            '    - `organize`\n\n'
            'Learn more about these part keywords by running '
            '`snapcraft help plugins`'
        )
        self.assertThat(exception.output, Contains(expected_help))

    def test_classic_confinement(self):
        project_dir = 'classic-build'

        # The first run should fail as the environment variable is not
        # set but we can only test this on clean systems.
        if not os.path.exists(os.path.join(
                os.path.sep, 'snap', 'core', 'current')):
            try:
                self.run_snapcraft(['stage'], project_dir)
            except subprocess.CalledProcessError:
                pass
            else:
                self.fail(
                    'This should fail as SNAPCRAFT_SETUP_CORE is not set')

        # Now we set the required environment variable
        self.useFixture(fixtures.EnvironmentVariable(
                'SNAPCRAFT_SETUP_CORE', '1'))

        self.run_snapcraft(['stage'], project_dir)
        self.assertThat(os.path.join(self.stage_dir, 'bin', 'hello-classic'),
                        FileExists())

    def test_staging_libc_links(self):
        project_dir = 'staging_links_to_libc'

        # First, stage libc6-dev via stage-packages
        self.run_snapcraft(['stage', 'from-package'], project_dir)

        # Now tar up the staging area
        subprocess.check_call(['tar', 'cf', 'stage.tar', 'stage/'])

        # Now attempt to stage the tarred staging area once again. This should
        # not conflict.
        try:
            self.run_snapcraft(['stage', 'from-tar'], project_dir)
        except subprocess.CalledProcessError as e:
            if 'have the following file paths in common' in e.output:
                self.fail('Parts unexpectedly conflicted')
            else:
                raise

    def symlinks_to_libc_should_build(self):
        """Regression test for LP: #1665089"""

        # This will fail to build if the libc symlinks are missing
        self.run_snapcraft('stage', 'use_libc_dl')

    def test_stage_with_file_to_check_for_collisions_not_build(self):
        """Regression test for LP: #1660696"""
        # This will fail if we try to check for collisions even in parts that
        # haven't been build.
        self.run_snapcraft('stage', 'stage-with-two-equal-files')
