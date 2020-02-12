# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2020 Canonical Ltd
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

import subprocess
from textwrap import dedent

from testtools.matchers import Equals, Contains, FileExists

from tests import integration


class StatusTestCase(integration.StoreTestCase):
    def test_status_with_login_wrong_snap(self):
        self.addCleanup(self.logout)
        self.login()

        error = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, ["status", "mysnap"]
        )

        self.assertThat(
            str(error.output),
            Contains(
                dedent(
                    """\
            Snap 'mysnap' was not found.

            Recommended resolution:
            Ensure you have proper access rights for 'mysnap'.
        """
                )
            ),
        )

    def test_status_with_login_bad_snap_with_arch(self):
        self.addCleanup(self.logout)
        self.login()

        error = self.assertRaises(
            subprocess.CalledProcessError,
            self.run_snapcraft,
            ["status", "mysnap", "--arch=i386"],
        )

        self.assertThat(
            str(error.output),
            Contains(
                dedent(
                    """\
            Snap 'mysnap' for architecture 'i386' was not found.

            Recommended resolution:
            Ensure you have proper access rights for 'mysnap'.
            Also ensure the correct architecture was used.
        """
                )
            ),
        )

    def test_status_fake_store(self):
        if not self.is_store_fake():
            self.skipTest("This test only works in the fake store")

        self.addCleanup(self.logout)
        self.login()

        output = self.run_snapcraft(["status", "basic"])
        expected = dedent(
            """\
            Track    Arch    Channel    Version    Revision    Notes    Expires at
            latest   all     stable     -          -           -
                             candidate  -          -           -
                             beta       1.1-amd64  6           -
                             edge       1.0-i386   3           -
                             edge/test  1.1-i386   9           -        2019-05-30T01:17:06.465504
                     amd64   stable     1.0-amd64  2           -
                             candidate  -          -           -
                             beta       1.1-amd64  4           -
                             edge       ^          ^           -
                             edge/test  1.1-amd64  10          -        2019-05-30T01:17:06.465504
                     i386    stable     -          -           -
                             candidate  -          -           -
                             beta       1.1-amd64  6           -
                             edge       1.0-i386   3           -
                             edge/test  1.1-i386   9           -        2019-05-30T01:17:06.465504
            """
        )
        self.assertThat(output, Contains(expected))

    def test_status_no_id(self):
        if not self.is_store_fake():
            self.skipTest("This test only works in the fake store")

        self.addCleanup(self.logout)
        self.login()

        error = self.assertRaises(
            subprocess.CalledProcessError, self.run_snapcraft, ["status", "no-id"]
        )
        self.assertThat(
            error.output, Contains("Failed to get snap ID for snap 'no-id'")
        )

    def test_status_staging_store(self):
        if not self.is_store_staging():
            self.skipTest("This test only works in the staging store")

        self.addCleanup(self.logout)
        self.login()

        # Build a random snap, register, push and release it.
        name = self.get_unique_name()
        version = self.get_unique_version()
        self.run_snapcraft(["init"])
        self.update_name_and_version(name, version)
        self.run_snapcraft("snap")
        snap_path = "{}_{}_{}.snap".format(name, version, self.deb_arch)
        self.assertThat(snap_path, FileExists())
        self.register(name)
        self.assertThat(self.push(snap_path, release="beta"), Equals(0))

        output = self.run_snapcraft(["status", name])
        expected = dedent(
            """\
            Track    Arch    Channel    Version                           Revision    Notes
            latest   {arch}   stable     -                                 -           -
                             candidate  -                                 -           -
                             beta       {version}  1           -
                             edge       ^                                 ^           -
            """
        ).format(arch=self.deb_arch, version=version)
        self.assertThat(output, Contains(expected))
