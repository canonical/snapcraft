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
import re
import subprocess

from testtools.matchers import Contains, FileExists, MatchesRegex

from tests import integration


class ReleaseTestCase(integration.StoreTestCase):
    def test_release_without_login(self):
        error = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft,
            ["release", "test-snap", "19", "beta"],
        )
        self.assertIn(
            'No valid credentials found. Have you run "snapcraft ' 'login"?',
            str(error.output),
        )

    def test_release_with_login(self):
        self.addCleanup(self.logout)
        self.login()

        # Change to a random name and version.
        name = self.get_unique_name()
        version = self.get_unique_version()
        self.copy_project_to_cwd("basic")
        self.update_name_and_version(name, version)

        self.run_snapcraft("snap")

        # Register the snap
        self.register(name)
        # Upload the snap
        snap_file_path = "{}_{}_{}.snap".format(name, version, "all")
        self.assertThat(os.path.join(snap_file_path), FileExists())

        output = self.run_snapcraft(["upload", snap_file_path])
        expected = r".*Ready to release!.*"
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))

        # Release it
        output = self.run_snapcraft(["release", name, "1", "edge"])
        expected = r".*The \'edge\' channel is now open.*"
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))

    def test_release_to_channel_without_permission(self):
        if not self.is_store_fake():
            self.skipTest("The real store won't return the proper response")

        self.addCleanup(self.logout)
        self.login()

        # Change to a random name and version.
        name = self.get_unique_name()
        version = self.get_unique_version()
        self.copy_project_to_cwd("basic")
        self.update_name_and_version(name, version)

        self.run_snapcraft("snap")

        # Register the snap
        self.register(name)
        # Upload the snap
        snap_file_path = "{}_{}_{}.snap".format(name, version, "all")
        self.assertThat(os.path.join(snap_file_path), FileExists())

        output = self.run_snapcraft(["push", snap_file_path])
        expected = r".*Ready to release!.*"
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))

        # Attempt to release it
        error = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft,
            ["release", name, "1", "no-permission"],
        )
        self.assertThat(
            error.output,
            Contains(
                "Received 403: Lacking permission to release to channel(s) "
                "'no-permission'"
            ),
        )

    def test_release_internal_error(self):
        if not self.is_store_fake():
            self.skipTest("The real store won't return the proper response")

        self.addCleanup(self.logout)
        self.login()

        # Change to a random name and version.
        name = self.get_unique_name()
        version = self.get_unique_version()
        self.copy_project_to_cwd("basic")
        self.update_name_and_version(name, version)

        self.run_snapcraft("snap")

        # Register the snap
        self.register(name)
        # Upload the snap
        snap_file_path = "{}_{}_{}.snap".format(name, version, "all")
        self.assertThat(os.path.join(snap_file_path), FileExists())

        output = self.run_snapcraft(["push", snap_file_path])
        expected = r".*Ready to release!.*"
        self.assertThat(output, MatchesRegex(expected, flags=re.DOTALL))

        # Attempt to release it
        error = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft,
            ["release", name, "1", "bad-channel"],
        )
        self.assertThat(
            error.output,
            Contains(
                "Snap Store encountered an error while processing your request: "
                "internal server error (code 500).\nThe operational status of the "
                "Snap Store can be checked at https://status.snapcraft.io/"
            ),
        )
