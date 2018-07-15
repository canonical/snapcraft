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

from tests import integration


class LoginLogoutTestCase(integration.StoreTestCase):
    def test_successful_login(self):
        self.addCleanup(self.logout)
        self.login(expect_success=True)

    def test_failed_login(self):
        self.login(
            "snapcraft-test+user@canonical.com", "wrongpassword", expect_success=False
        )

    def test_login_with(self):
        pass
