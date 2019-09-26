# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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
import re

from testtools.matchers import FileExists, MatchesRegex

from tests import integration, skip


# FIXME: move the store tests to spread.


class PromoteTestCase(integration.StoreTestCase):
    @skip.skip_unless_codename("xenial", "test designed for xenial")
    def test_promote_with_login(self):
        self.addCleanup(self.logout)
        self.login()

        # Change to a random name and version.
        name = self.get_unique_name()
        version = self.get_unique_version()

        self.run_snapcraft(["init"])

        self.update_name_and_version(name, version)

        self.run_snapcraft("snap")

        # Register the snap
        self.register(name)

        # Push and release the snap
        snap_file_path = "{}_{}_{}.snap".format(name, version, self.deb_arch)
        self.assertThat(os.path.join(snap_file_path), FileExists())
        output = self.run_snapcraft(["push", "--release", "edge/test", snap_file_path])

        # Promote it
        output = self.run_snapcraft(
            [
                "promote",
                name,
                "--from-channel",
                "edge/test",
                "--to-channel",
                "beta",
                "--yes",
            ]
        )
        expected = r".*The 'beta' channel is now open.*"
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))
