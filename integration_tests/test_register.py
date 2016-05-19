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
from snapcraft.tests import fixture_setup


class RegisterTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()
        self.useFixture(fixture_setup.StagingStore())

    # FIXME: The store doesn't provide a way to unregister a name *and*
    # registrations are rate-limited for a given user. We work around that by
    # creating unique names and assuming we only run against staging or local
    # dev instances -- vila 2016-04-08
    def test_successful_register(self):
        self.login(expect_success=True)
        self.addCleanup(self.logout)
        uniq_name = 'delete-me-{}'.format(str(uuid.uuid4().int)[:32])
        output = self.run_snapcraft(['register-name', uniq_name])
        self.assertIn('Congrats! You\'re now the publisher'
                      ' for "{}"'.format(uniq_name), output)

    def test_register_without_login(self):
        output = self.run_snapcraft(['register-name', 'foobar'])
        self.assertIn('Registration failed.', output)
        self.assertIn('No valid credentials found. Have you run "snapcraft '
                      'login"?', output)

    def test_register_registered_name(self):
        self.login(expect_success=True)
        self.addCleanup(self.logout)
        output = self.run_snapcraft(['register-name', 'femto'])
        self.assertIn('Registration failed.', output)
