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

from testtools.matchers import Contains, FileExists, Not

from tests import integration


class PrimeKeywordTestCase(integration.TestCase):
    def test_prime_filter(self):
        self.run_snapcraft(["prime", "prime-keyword"], "prime-filter")

        # Verify that only the `prime1` file made it into prime (i.e. `prime2`
        # was filtered out).
        self.assertThat(os.path.join(self.prime_dir, "prime1"), FileExists())
        self.assertThat(os.path.join(self.prime_dir, "prime2"), Not(FileExists()))

    def test_snap_filter_is_deprecated(self):
        output = self.run_snapcraft(["prime", "snap-keyword"], "prime-filter")

        # Verify that the `snap` keyword is deprecated.
        self.assertThat(
            output,
            Contains(
                "DEPRECATED: The 'snap' keyword has been replaced by 'prime'."
                "\nSee http://snapcraft.io/docs/deprecation-notices/dn1 "
                "for more information."
            ),
        )

        # Verify that only the `snap1` file made it into prime (i.e. `snap2`
        # was filtered out).
        self.assertThat(os.path.join(self.prime_dir, "snap1"), FileExists())
        self.assertThat(os.path.join(self.prime_dir, "snap2"), Not(FileExists()))
