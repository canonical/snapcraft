# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

from snapcraft.tests import integration


class RegisteredTestCase(integration.StoreTestCase):

    def test_registered_status_and_privacy(self):
        self.login()

        # Make sure 'snap_public' is listed before 'snap_private',
        # thus the 'a/b' suffixes (alphabetical order).
        snap_public = self.get_unique_name('a')
        self.register(snap_public)
        snap_private = self.get_unique_name('b')
        self.register(snap_private, private=True)

        expected_snaps = [
            (snap_public, 'public', '-', '-'),
            (snap_private, 'private', '-', '-'),
        ]
        self.list_registered(expected_snaps)
