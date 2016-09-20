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

import integration_tests
import uuid


class SignBuildTestCase(integration_tests.StoreTestCase):

    def setUp(self):
        super().setUp()

    def test_successful_sign_build(self):
        project_dir = 'basic'
        self.addCleanup(self.logout)
        self.login()

        unique_id = uuid.uuid4().int
        new_name = 'u1test-{}'.format(unique_id)
        new_version = str(unique_id)[:32]
        project_dir = self.update_name_and_version(
            project_dir, new_name, new_version)
        self.run_snapcraft('snap', project_dir)

        self.register(new_name)
        snap_file_path = '{}_{}_{}.snap'.format(new_name, new_version, 'all')
        output = self.sign_build(snap_file_path)
        expected = 'Assertion {} saved to disk.'.format(snap_file_path)
        self.assertThat(output, expected)
