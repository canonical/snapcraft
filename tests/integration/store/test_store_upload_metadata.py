# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2017-2020 Canonical Ltd
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

import fixtures
from testtools.matchers import Contains, FileExists

from tests import integration


class UploadMetadataTestCase(integration.StoreTestCase):
    def _build_snap(self):
        """Build a snap file and return its name and path."""
        name = self.get_unique_name()
        version = self.get_unique_version()
        self.copy_project_to_cwd("basic")
        self.update_name_and_version(name, version)

        self.run_snapcraft("snap")

        snap_file_path = "{}_{}_{}.snap".format(name, version, "all")
        self.assertThat(os.path.join(snap_file_path), FileExists())
        return name, snap_file_path

    def test_with_login(self):
        self.addCleanup(self.logout)
        self.login()

        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG", "yes")
        )

        # Build and register the snap
        name, snap_file_path = self._build_snap()
        self.register(name)

        # Upload the snap
        output = self.run_snapcraft(["upload", snap_file_path])

        # Now upload the metadata
        output = self.run_snapcraft(["upload-metadata", snap_file_path])
        expected = "Uploading metadata to the Store (force=False)"
        self.assertThat(output, Contains(expected))
        expected = "The metadata has been uploaded"
        self.assertThat(output, Contains(expected))

    def test_no_upload_needed_first(self):
        self.addCleanup(self.logout)
        self.login()

        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG", "yes")
        )

        # Build and register the snap
        name, snap_file_path = self._build_snap()
        self.register(name)

        # Now upload the metadata
        output = self.run_snapcraft(["upload-metadata", snap_file_path])
        expected = "Uploading metadata to the Store (force=False)"
        self.assertThat(output, Contains(expected))
        expected = "The metadata has been uploaded"
        self.assertThat(output, Contains(expected))
