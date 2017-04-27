# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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
import re
import subprocess

from testtools.matchers import Contains, MatchesRegex

import integration_tests


class RegisterTestCase(integration_tests.StoreTestCase):

    def test_successful_registration(self):
        self.login()
        snap_name = self.get_unique_name()
        self.register(snap_name)

    def test_successful_private_registration(self):
        self.login()
        snap_name = self.get_unique_name()
        self.register(snap_name, private=True)

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
        self.assertThat(str(error.output), Contains('register-name'))

    def test_registration_of_already_owned_name(self):
        self.login()
        self.addCleanup(self.logout)
        if os.getenv('TEST_STORE', 'fake') != 'fake':
            snap_name = self.get_unique_name()
            self.register(snap_name)
        else:
            snap_name = self.test_store.already_owned_snap_name

        # The snap name is already registered and you are the owner.
        error = self.assertRaises(
            subprocess.CalledProcessError, self.register, snap_name)
        expected = 'You already own the name {0!r}'.format(snap_name)
        self.assertThat(str(error.output), Contains(expected))
        self.assertThat(str(error.output), Contains('register-name'))

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
        self.assertThat(str(error.output), Contains('register-name'))

    def test_registration_of_invalid_name(self):
        self.login()
        name = 'test_invalid'
        error = self.assertRaises(
            subprocess.CalledProcessError, self.register, name)

        expected = (
            'The name {!r} is not valid. It can only contain dashes, numbers '
            'and lowercase ascii letters.'.format(name))
        self.assertThat(str(error.output), Contains(expected))
        self.assertThat(str(error.output), Contains('register-name'))

    def test_registration_of_too_long_name(self):
        self.login()
        name = 'name-too-l{}ng'.format('o' * 40)
        error = self.assertRaises(
            subprocess.CalledProcessError, self.register, name)

        expected = (
            'The name {} should not be longer than 40 characters.'
            .format(name))
        self.assertThat(str(error.output), Contains(expected))
        self.assertThat(str(error.output), Contains('register-name'))

    def test_registrations_in_a_row_fail_if_too_fast(self):
        # This test has a potential to fail if working off a slow
        # network.
        self.login()

        error = None
        for idx in range(self.test_store.register_count_limit + 1):
            snap_name = self.get_unique_name('test-too-fast-{}'.format(idx))
            try:
                self.register(snap_name, wait=False)
            except subprocess.CalledProcessError as exc:
                error = exc
                break

        self.assertIsNotNone(error, 'An error must be raised.')
        expected = (
            '.*You must wait \d+ seconds before trying to register your '
            'next snap.*')
        self.assertThat(
            str(error.output), MatchesRegex(expected, flags=re.DOTALL))
