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

import uuid

import integration_tests


class RegisteredTestCase(integration_tests.StoreTestCase):

    def test_registered_status_and_privacy(self):
        self.login()

        # Make sure 'snap_public' is listed before 'snap_private',
        # thus the 'a/b' suffixes (alphabetical order).
        snap_public = 'u1test-a-{}'.format(uuid.uuid4().int)
        self.register(snap_public)
        snap_private = 'u1test-b-{}'.format(uuid.uuid4().int)
        self.register(snap_private, private=True)

        expected_snaps = [
            (snap_public, 'public', '-', '-'),
            (snap_private, 'private', '-', '-'),
        ]
        self.list_registered(expected_snaps)
