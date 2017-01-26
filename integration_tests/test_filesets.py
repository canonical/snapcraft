# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015, 2016, 2017 Canonical Ltd
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

import integration_tests

from testtools.matchers import (
    DirExists,
    FileContains,
    Not,
)


class FilesetsTestCase(integration_tests.TestCase):

    def test_filesets(self):
        self.run_snapcraft('snap', 'simple-make-filesets')

        expected_dirs = (
            os.path.join('stage', 'share'),
            os.path.join('stage', 'bin'),
            os.path.join('prime', 'bin'),
        )
        for expected_dir in expected_dirs:
            self.assertThat(
                expected_dir,
                DirExists())

        self.assertThat(
            os.path.join('prime', 'share'),
            Not(DirExists()))

        expected_files = (
            (os.path.join('stage', 'share', 'share1'), 'share1\n'),
            (os.path.join('stage', 'share', 'share2'), 'share2\n'),
            (os.path.join('stage', 'bin', 'bin1'), 'bin1\n'),
            (os.path.join('stage', 'bin', 'bin2'), 'bin2\n'),
            (os.path.join('prime', 'bin', 'bin1'), 'bin1\n'),
            (os.path.join('prime', 'bin', 'bin2'), 'bin2\n'),
        )
        for path, content in expected_files:
            self.assertThat(path, FileContains(content))
