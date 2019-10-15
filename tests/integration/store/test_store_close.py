# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2019 Canonical Ltd
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

from testtools.matchers import Equals, FileExists

from tests import integration


class ChannelClosingTestCase(integration.StoreTestCase):
    def test_missing_permission(self):
        self.addCleanup(self.logout)
        self.login()
        expected = (
            "Make sure the logged in account has upload permissions on "
            "'missing' in series '16'."
        )
        status = self.close("missing", "beta", expected=expected)
        self.assertThat(status, Equals(2))

    def test_close_channel(self):
        self.addCleanup(self.logout)
        self.login()

        # Change to a random name and version when not on the fake store.
        if not self.is_store_fake():
            name = self.get_unique_name()
            version = self.get_unique_version()
        # If not, keep the name that is faked in our fake account.
        else:
            name = "basic"
            version = "1.0"

        self.copy_project_to_cwd("basic")
        self.update_name_and_version(name, version)

        self.run_snapcraft("snap")

        # Register the snap
        self.register(name)

        # Upload the snap
        snap_file_path = "{}_{}_{}.snap".format(name, version, "all")
        self.assertThat(os.path.join(snap_file_path), FileExists())

        self.assertThat(self.push(snap_file_path, release="edge,beta"), Equals(0))

        expected = "The beta channel is now closed."
        status = self.close(name, "beta", expected=expected)
        self.assertThat(status, Equals(0))
