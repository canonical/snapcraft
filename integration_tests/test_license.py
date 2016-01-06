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

from testtools.matchers import (
    DirExists,
    FileContains,
    FileExists,
)

import integration_tests


class LicenseTestCase(integration_tests.TestCase):

    def test_license(self):
        project_dir = 'license'
        self.run_snapcraft('strip', project_dir)

        hooks_dir = os.path.join(project_dir, 'snap', 'meta', 'hooks')
        self.assertThat(hooks_dir, DirExists())
        license_hook = os.path.join(hooks_dir, 'license')
        self.assertThat(license_hook, FileExists())
        self.assertThat(license_hook, FileContains(
            'This is a license.\n'))
