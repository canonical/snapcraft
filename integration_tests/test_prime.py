# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016 Canonical Ltd
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

from testtools.matchers import (
    FileExists,
    Not,
)

import snapcraft
import integration_tests


class PrimeTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()
        self.deb_arch = snapcraft.ProjectOptions().deb_arch

    def test_prime_includes_stage_fileset(self):
        project_dir = 'prime-from-stage'

        self.run_snapcraft('prime', project_dir)
        self.assertThat(
            os.path.join(project_dir, 'prime', 'without-a'),
            FileExists())
        self.assertThat(
            os.path.join(project_dir, 'prime', 'without-b'),
            Not(FileExists()))
        self.assertThat(
            os.path.join(project_dir, 'prime', 'without-c'),
            FileExists())

    def test_prime_includes_stage_excludes_fileset(self):
        project_dir = 'prime-from-stage'

        self.run_snapcraft('prime', project_dir)
        self.assertThat(
            os.path.join(project_dir, 'prime', 'with-a'),
            Not(FileExists()))
        self.assertThat(
            os.path.join(project_dir, 'prime', 'with-b'),
            FileExists())
        self.assertThat(
            os.path.join(project_dir, 'prime', 'with-c'),
            FileExists())
