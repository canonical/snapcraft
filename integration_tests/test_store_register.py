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

import re
import subprocess
import time
import uuid

from testtools.matchers import Contains, MatchesRegex

import integration_tests


class RegisterTestCase(integration_tests.StoreTestCase):

    def test_successful_registration(self):
        self.login()
        snap_name = 'u1test{}'.format(uuid.uuid4().int)
        self.register(snap_name)

    def test_failed_registration_already_registered(self):
        self.login()
        # The snap name is already registered.
        error = self.assertRaises(
            subprocess.CalledProcessError,
            self.register, 'test-already-registered-snap-name')
        expected = (
            'The name \'test-already-registered-snap-name\' is already '
            'taken.'
            '\n\n'
            'We can if needed rename snaps to ensure they match the '
            'expectations of most users. If you are the publisher most '
            'users expect for \'test-already-registered-snap-name\' then '
            'claim the name at')
        self.assertThat(str(error.output), Contains(expected))
        self.assertThat(str(error.output), Contains('register-name-dispute'))

    def test_registration_of_reserved_name(self):
        self.login()
        # The snap name is already registered.
        error = self.assertRaises(
            subprocess.CalledProcessError,
            self.register, self.test_store.reserved_snap_name)
        expected = (
            'The name {0!r} is reserved.\n\n'
            'If you are the publisher most users expect for {0!r} '
            'then please claim the name at').format(
                self.test_store.reserved_snap_name)
        self.assertThat(str(error.output), Contains(expected))
        self.assertThat(str(error.output), Contains('register-name-dispute'))

    def test_registrations_in_a_row_fail_if_too_fast(self):
        # Wait after the registration attempts, so the following registrations
        # don't get the error.
        self.addCleanup(time.sleep, self.test_store.register_delay)
        # This test has a potential to fail if working off a slow
        # network.
        self.login()
        snap_name_1 = 'good-snap{}'.format(uuid.uuid4().int)
        snap_name_2 = 'test-too-fast{}'.format(uuid.uuid4().int)

        self.register(snap_name_1, wait=False)

        error = self.assertRaises(
            subprocess.CalledProcessError,
            self.register, snap_name_2, wait=False)
        expected = (
            '.*You must wait \d+ seconds before trying to register your '
            'next snap.*')
        self.assertThat(
            str(error.output), MatchesRegex(expected, flags=re.DOTALL))
