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

import os.path
import shutil

import fixtures
from testtools.matchers import Equals

from tests import integration


class RegisterKeyTestCase(integration.StoreTestCase):
    def setUp(self):
        super().setUp()
        keys_dir = os.path.join(os.path.dirname(__file__), "keys")
        temp_keys_dir = os.path.join(self.path, ".snap", "gnupg")
        shutil.copytree(keys_dir, temp_keys_dir)
        self.useFixture(fixtures.EnvironmentVariable("SNAP_GNUPG_HOME", temp_keys_dir))

    def test_successful_key_registration(self):
        if not self.is_store_fake():
            # https://bugs.launchpad.net/bugs/1621441
            self.skipTest(
                "Cannot register test keys against staging/production until "
                "we have a way to delete them again."
            )
        self.assertThat(self.register_key("default"), Equals(0))
        self.addCleanup(self.logout)
        self.login()
        self.assertThat(
            self.list_keys(
                [
                    (
                        True,
                        "default",
                        "2MEtiEuR7eCBUocloPokPhqPSTpkj7Kk"
                        "TPQNZYOiZshFHdfzxlEhc8ITzpHq5azq",
                    ),
                    (
                        False,
                        "another",
                        "OR59L-ompOW_CHbQ4pNDW5B-7BVY_V3Q"
                        "kPu5-7uOFrZEEEAoI4h3yc_RQv3qmAFJ",
                    ),
                ]
            ),
            Equals(0),
        )

    def test_failed_login_for_key_registration(self):
        status = self.register_key(
            "default",
            "snapcraft-test+user@canonical.com",
            "wrongpassword",
            expect_success=False,
        )
        self.assertNotEqual(0, status)
