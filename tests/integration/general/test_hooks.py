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

from testtools.matchers import Contains, FileContains, FileExists, Not

from tests import integration


class HookTestCase(integration.TestCase):
    def test_hooks(self):
        self.run_snapcraft("prime", "hooks")

        for hook in ("configure", "another-hook"):
            # Assert that the hooks as supplied to snapcraft was copied into
            # the snap.
            self.assertThat(
                os.path.join(self.prime_dir, "snap", "hooks", hook), FileExists()
            )

            # Assert that the real hooks were generated as well (they're just
            # wrappers that call the one given to snapcraft).
            self.assertThat(
                os.path.join(self.prime_dir, "meta", "hooks", hook), FileExists()
            )

        # Assert that the wrapper execs the correct thing
        self.assertThat(
            os.path.join(self.prime_dir, "meta", "hooks", "another-hook"),
            FileContains(matcher=Contains('exec "$SNAP/snap/hooks/{}"'.format(hook))),
        )

        # Assert that the configure hook doesn't have a wrapper
        self.assertThat(
            os.path.join(self.prime_dir, "meta", "hooks", "configure"),
            Not(FileContains(matcher=Contains("exec".format(hook)))),
        )
