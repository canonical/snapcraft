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

from testtools.matchers import Equals

from tests import integration


class GatedTestCase(integration.StoreTestCase):
    def setUp(self):
        super().setUp()
        if not self.is_store_fake():
            self.skipTest(
                "Right combination of snaps and IDs is not available in real stores."
            )

    def test_gated_success(self):
        self.addCleanup(self.logout)
        self.login()
        validations = [("snap-1", "3"), ("snap-2", "5")]
        self.assertThat(self.gated("core", validations), Equals(0))

    def test_gated_no_login_failure(self):
        self.assertThat(
            self.gated("core", expected_output='Have you run "snapcraft login'),
            Equals(2),
        )

    def test_gated_unknown_snap_failure(self):
        self.addCleanup(self.logout)
        self.login()
        self.assertThat(
            self.gated("unknown", expected_output="Snap 'unknown' was not found"),
            Equals(2),
        )

    def test_gated_no_validations(self):
        self.addCleanup(self.logout)
        self.login()
        snap_name = "test-snap-with-no-validations"
        self.assertThat(
            self.gated(
                snap_name,
                expected_output=(
                    "There are no validations for snap '{}'".format(snap_name)
                ),
            ),
            Equals(0),
        )
