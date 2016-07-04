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
import subprocess
import uuid

from testtools.matchers import EndsWith

import integration_tests
from snapcraft.tests import fixture_setup


class RegisterTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()
        if not os.getenv('TEST_USER_PASSWORD', None):
            self.useFixture(fixture_setup.FakeStore())
        else:
            self.useFixture(fixture_setup.StagingStore())

    def test_successful_registration(self):
        self.login(expect_success=True)
        snap_name = 'u1test{}'.format(uuid.uuid4().int)
        self.register(snap_name)

    def test_failed_registration_already_registered(self):
        self.login(expect_success=True)
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
            'claim the name at \'https://myapps.com/register-name-dispute/'
            '\'\n')
        self.assertThat(str(error.output), EndsWith(expected))
