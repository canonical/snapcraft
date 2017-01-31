# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016, 2017 Canonical Ltd
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
    DirExists,
    FileExists,
    Not
)

import integration_tests


class LibraryPrecedenceTestCase(integration_tests.TestCase):

    def test_snapped_library_takes_precedence_over_system(self):
        self.run_snapcraft('stage', 'fake-curl-library')
        self.run_snapcraft(['prime', 'main'])

        # Verify that, while the binary was primed, no library was pulled in.
        self.assertThat(
            os.path.join(self.prime_dir, 'bin', 'main'), FileExists())
        self.assertThat(
            os.path.join(self.prime_dir, 'lib'), Not(DirExists()))
        self.assertThat(
            os.path.join(self.prime_dir, 'usr'), Not(DirExists()))

        # Prime the rest of the way.
        self.run_snapcraft('prime')

        # Now verify the lib we got was the one from the snap, not from the
        # system.
        self.assertThat(
            os.path.join(self.prime_dir, 'lib', 'libcurl.so'),
            FileExists())
        self.assertThat(
            os.path.join(self.prime_dir, 'usr'), Not(DirExists()))
