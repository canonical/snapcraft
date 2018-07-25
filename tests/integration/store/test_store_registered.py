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


class RegisteredTestCase(integration.StoreTestCase):
    def test_registered_status_and_privacy_private(self):
        self.login()

        snap_private = self.get_unique_name()
        self.register(snap_private, private=True)

        expected_snaps = [(snap_private, "private", "-", "-")]
        self.list_registered(expected_snaps)

    def test_registered_status_and_privacy_public(self):
        self.login()

        snap_public = self.get_unique_name()
        self.register(snap_public)

        expected_snaps = [(snap_public, "public", "-", "-")]
        self.list_registered(expected_snaps)
