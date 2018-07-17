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

from testtools.matchers import HasPermissions

from tests import integration


class ExportLoginTestCase(integration.StoreTestCase):
    def test_successful_export(self):
        self.export_login("exported", expect_success=True)

        # Verify that the exported login is only readable by the owner
        self.assertThat("exported", HasPermissions("0400"))

    def test_failed_export(self):
        self.export_login(
            "exported",
            "snapcraft-test+user@canonical.com",
            "wrongpassword",
            expect_success=False,
        )
