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

import uuid

from testtools.matchers import FileExists

import integration_tests


class ChannelClosingTestCase(integration_tests.StoreTestCase):

    def test_missing_login(self):
        expected = 'run "snapcraft login"'
        status = self.close('basic', 'beta', expected=expected)
        self.assertEqual(1, status)

    def test_missing_permission(self):
        self.addCleanup(self.logout)
        self.login()
        expected = (
            'Make sure the logged in account has upload permissions on '
            "'missing' in series '16'.")
        status = self.close('missing', 'beta', expected=expected)
        self.assertEqual(1, status)

    def test_close_channel(self):
        self.addCleanup(self.logout)
        self.login()

        # Build a random snap, register, push and release it.
        unique_id = uuid.uuid4().int
        name = 'u1test-{}'.format(unique_id)
        version = str(unique_id)[:32]
        self.copy_project_to_cwd('basic')
        self.update_name_and_version(name, version)
        self.run_snapcraft('snap')
        snap_path = '{}_{}_{}.snap'.format(name, version, 'all')
        self.assertThat(snap_path, FileExists())
        self.register(name)
        self.assertEqual(0, self.push(snap_path, release='edge,beta'))

        expected = 'The beta channel is now closed.'
        status = self.close(name, 'beta', expected=expected)
        self.assertEqual(0, status)
