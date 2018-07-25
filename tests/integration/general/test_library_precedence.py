# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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

from testtools.matchers import DirExists, FileExists, Not

from tests import integration


class LibraryPrecedenceTestCase(integration.TestCase):
    def test_snapped_library_takes_precedence_over_system(self):
        self.run_snapcraft("stage", "fake-curl-library")
        self.run_snapcraft(["prime", "main", "fake-curl"])

        # We will have everything in, given that we require dependencies
        # to be primed.
        self.assertThat(os.path.join(self.prime_dir, "bin", "main"), FileExists())
        self.assertThat(os.path.join(self.prime_dir, "lib"), DirExists())
        # If this exist, snapcraft brought libbcurl in from the host.
        self.assertThat(os.path.join(self.prime_dir, "usr"), Not(DirExists()))

        # Prime the rest of the way.
        self.run_snapcraft("prime")

        # Now verify the lib we got was the one from the snap, not from the
        # system.
        self.assertThat(os.path.join(self.prime_dir, "lib", "libcurl.so"), FileExists())
        self.assertThat(os.path.join(self.prime_dir, "usr"), Not(DirExists()))
