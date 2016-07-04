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

from testtools.matchers import FileExists

import integration_tests
from snapcraft.tests import fixture_setup


class DownloadTestCase(integration_tests.TestCase):

    def setUp(self):
        super().setUp()
        if not os.getenv('TEST_USER_PASSWORD', None):
            self.useFixture(fixture_setup.FakeStore())
        else:
            self.skipTest('There is no ubuntu-core snap in the staging server')
            # TODO add the snap to the staging server.
        self.login()

    def test_download_os_snap(self):
        project_dir = 'kernel-download'
        self.run_snapcraft('pull', project_dir)
        self.assertThat(
            os.path.join(project_dir, 'os.snap'), FileExists())
