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
import shutil

import fixtures
from testtools.matchers import Equals

from tests import integration


class ValidateTestCase(integration.StoreTestCase):
    def setUp(self):
        super().setUp()
        if not self.is_store_fake():
            self.skipTest(
                "Right combination of snaps and IDs is not available in real stores."
            )

        keys_dir = os.path.join(os.path.dirname(__file__), "keys")
        temp_keys_dir = os.path.join(self.path, ".snap", "gnupg")
        shutil.copytree(keys_dir, temp_keys_dir)
        self.useFixture(fixtures.EnvironmentVariable("SNAP_GNUPG_HOME", temp_keys_dir))

    def test_validate_success(self):
        self.addCleanup(self.logout)
        self.login()
        self.assertThat(self.validate("core", ["core=3", "test-snap=4"]), Equals(0))

    def test_validate_unknown_snap_failure(self):
        self.addCleanup(self.logout)
        self.login()
        self.assertThat(
            self.validate(
                "unknown",
                ["core=3", "test-snap=4"],
                expected_error="Snap 'unknown' was not found.",
            ),
            Equals(2),
        )

    def test_validate_bad_argument(self):
        self.addCleanup(self.logout)
        self.login()
        self.assertThat(
            self.validate(
                "core", ["core=foo"], expected_error="format must be name=revision"
            ),
            Equals(2),
        )

    def test_validate_no_login_failure(self):
        self.assertThat(
            self.validate(
                "core",
                ["core=3", "test-snap=4"],
                expected_error='Have you run "snapcraft login"',
            ),
            Equals(2),
        )
