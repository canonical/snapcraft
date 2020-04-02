# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2020 Canonical Ltd
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

from testtools.matchers import Equals, FileExists

from tests import integration


class StatusTestCase(integration.StoreTestCase):
    def test_status_fake_store(self):
        if not self.is_store_fake():
            self.skipTest("This test only works in the fake store")

        self.addCleanup(self.logout)
        self.login()

        # We only care about the contract with the store. Fine grained CLI
        # output is tested through unit tests.
        self.run_snapcraft(["status", "basic"])

    def test_status_staging_store(self):
        if not self.is_store_staging():
            self.skipTest("This test only works in the staging store")

        self.addCleanup(self.logout)
        self.login()

        # Build a random snap, register, push and release it.
        name = self.get_unique_name()
        version = self.get_unique_version()
        self.run_snapcraft(["init"])
        self.update_name_and_version(name, version)
        self.run_snapcraft("snap")
        snap_path = "{}_{}_{}.snap".format(name, version, self.deb_arch)
        self.assertThat(snap_path, FileExists())
        self.register(name)
        self.assertThat(self.push(snap_path, release="beta"), Equals(0))

        # We only care about the contract with the store. Fine grained CLI
        # output is tested through unit tests.
        self.run_snapcraft(["status", name])
