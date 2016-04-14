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

import fixtures

import integration_tests
from snapcraft.tests import fixture_setup


class RegisterTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()
        self.useFixture(fixture_setup.StagingStore())
        # FIXME: Switch to macaroons as soon as they are available for
        # register-name -- vila 2016-04-14
        self.useFixture(
            fixtures.EnvironmentVariable('SNAPCRAFT_WITH_MACAROONS', None))
#        self.useFixture(
#            fixtures.EnvironmentVariable('SNAPCRAFT_WITH_MACAROONS', '1'))
        self.login(expect_success=True)
        self.addCleanup(self.logout)

    # FIXME: The store doesn't provide a way to unregister a name *and*
    # registrations are rate-limited for a given user. We work around that by
    # creating unique names and assuming we only run against staging or local
    # dev instances -- vila 2016-04-08
    def test_successful_register(self):
        uniq_name = 'delete-me-{}'.format(str(uuid.uuid4().int)[:32])
        self.register_name(uniq_name)
