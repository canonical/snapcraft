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

import os

import integration_tests


class GatedTestCase(integration_tests.StoreTestCase):

    def setUp(self):
        if os.getenv('TEST_STORE', 'fake') != 'fake':
            self.skipTest('Right combination of snaps and IDs is not '
                          'available in real stores.')
        super().setUp()

    def test_gated_success(self):
        self.addCleanup(self.logout)
        self.login()
        validations = [('snap-1', '3'), ('snap-2', '5')]
        self.assertEqual(0, self.gated('ubuntu-core', validations))

    def test_gated_no_login_failure(self):
        self.assertEqual(1, self.gated(
            'ubuntu-core', expected_output='Have you run "snapcraft login'))

    def test_gated_unknown_snap_failure(self):
        self.addCleanup(self.logout)
        self.login()
        self.assertEqual(1, self.gated(
            'unknown', expected_output="Snap 'unknown' was not found"))

    def test_gated_no_validations(self):
        self.addCleanup(self.logout)
        self.login()
        self.assertEqual(0, self.gated(
            'basic',
            expected_output="There are no validations for snap 'basic'"))
