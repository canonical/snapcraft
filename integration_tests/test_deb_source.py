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

from testtools.matchers import (
    Equals,
    FileExists
)

import integration_tests


class DebSourceTestCase(integration_tests.TestCase):

    def test_stage_deb(self):
        self.copy_project_to_cwd('simple-deb')
        self.run_snapcraft(['stage', 'deb'])

        self.assertThat(
            os.path.join(self.stage_dir, 'bin', 'hello'),
            FileExists())
        self.assertThat(
            os.path.join(self.stage_dir, 'usr', 'bin', 'world'),
            FileExists())

    # Regression test for LP: #1634813
    def test_stage_deb_with_symlink(self):
        self.copy_project_to_cwd('simple-deb')
        self.run_snapcraft(['stage', 'deb-with-symlink'])

        target = os.path.join(self.stage_dir, 'target')
        symlink = os.path.join(self.stage_dir, 'symlink')
        self.assertThat(target, FileExists())
        self.assertThat(symlink, FileExists())
        self.assertTrue(os.path.islink(symlink))
        self.assertThat(os.readlink(symlink), Equals('target'))
