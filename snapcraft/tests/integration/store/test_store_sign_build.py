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
import fixtures
import shutil

from testtools.matchers import (
    Equals,
    Contains,
    FileContains,
    FileExists,
    Not,
)

from snapcraft.tests import integration


class SignBuildTestCase(integration.StoreTestCase):

    def setUp(self):
        super().setUp()
        keys_dir = os.path.join(os.path.dirname(__file__), 'keys')
        temp_keys_dir = os.path.join(self.path, '.snap', 'gnupg')
        shutil.copytree(keys_dir, temp_keys_dir)
        self.useFixture(fixtures.EnvironmentVariable(
            'SNAP_GNUPG_HOME', temp_keys_dir))
        self.project = 'basic'
        self.snap_path = '{}_0.1_all.snap'.format(self.project)
        self.snap_build_path = (
            os.path.splitext(self.snap_path)[0] + '.build')

    def test_unsuccessful_sign_build_no_login(self):
        self.run_snapcraft('snap', self.project)
        self.assertThat(self.snap_path, FileExists())

        status = self.sign_build(
            self.snap_path, local=True, expect_success=False)
        self.assertThat(status, Equals(2))
        self.assertThat(self.snap_build_path, Not(FileExists()))

    def test_successful_sign_build_local(self):
        self.addCleanup(self.logout)
        self.login()

        # Build a random snap and register it.
        name = self.get_unique_name()
        version = self.get_unique_version()
        self.copy_project_to_cwd(self.project)
        self.update_name_and_version(name, version)
        self.run_snapcraft('snap')
        snap_path = '{}_{}_{}.snap'.format(name, version, 'all')
        self.assertThat(snap_path, FileExists())
        self.register(name)

        status = self.sign_build(snap_path, local=True)
        self.assertThat(status, Equals(0))

        snap_build_path = os.path.splitext(snap_path)[0] + '.build'
        self.assertThat(snap_build_path, FileExists())
        self.assertThat(snap_build_path,
                        FileContains(matcher=Contains('type: snap-build')))

    def test_unsuccessful_sign_build_push_no_login(self):
        self.run_snapcraft('snap', self.project)
        self.assertThat(self.snap_path, FileExists())

        status = self.sign_build(self.snap_path, expect_success=False)
        self.assertThat(status, Equals(2))
        self.assertThat(self.snap_build_path, Not(FileExists()))

    def test_successful_sign_build_push(self):
        if not self.is_store_fake():
            # https://bugs.launchpad.net/bugs/1621441
            self.skipTest(
                'Cannot push signed assertion against staging/production '
                'until we have a way to delete them again.')

        self.run_snapcraft('snap', self.project)
        self.assertThat(self.snap_path, FileExists())

        self.assertThat(self.register_key('default'), Equals(0))
        self.addCleanup(self.logout)
        self.login()

        status = self.sign_build(self.snap_path)
        self.assertThat(status, Equals(0))
        self.assertThat(self.snap_build_path, FileExists())
        self.assertThat(self.snap_build_path,
                        FileContains(matcher=Contains('type: snap-build')))

    def test_legacy_build_filename_support(self):
        self.run_snapcraft('snap', self.project)
        self.assertThat(self.snap_path, FileExists())

        self.addCleanup(self.logout)
        self.login()

        legacy_build_path = self.snap_path + '-build'
        with open(legacy_build_path, 'a') as fd:
            fd.write('test')

        # Can identify legacy 'snap-build' instead of signing a new one..
        status = self.sign_build(
            self.snap_path, has_legacy_build=True, local=True)
        self.assertThat(status, Equals(0))
        self.assertThat(legacy_build_path, FileExists())
        self.assertThat(self.snap_build_path, Not(FileExists()))

        # Pushes the legacy snap-build filename if it exists.
        status = self.sign_build(
            self.snap_path, has_legacy_build=True)
        self.assertThat(status, Equals(0))
        self.assertThat(legacy_build_path, FileExists())
        self.assertThat(self.snap_build_path, Not(FileExists()))
