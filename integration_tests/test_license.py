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
        self.run_snapcraft('prime', project_dir)

        meta_dir = os.path.join(project_dir, 'prime', 'meta')
        self.assertThat(meta_dir, DirExists())
        license_asset = os.path.join(meta_dir, 'license.txt')
        self.assertThat(license_asset, FileExists())
        self.assertThat(license_asset, FileContains(
            'This is a license.\n'))
