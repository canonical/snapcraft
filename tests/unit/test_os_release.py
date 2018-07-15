# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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

from textwrap import dedent

from testtools.matchers import Equals

from snapcraft.internal import os_release, errors
from tests import unit


class OsReleaseTestCase(unit.TestCase):
    def _write_os_release(self, contents):
        path = "os-release"
        with open(path, "w") as f:
            f.write(contents)
        return path

    def test_blank_lines(self):
        release = os_release.OsRelease(
            os_release_file=self._write_os_release(
                dedent(
                    """\
                NAME="Arch Linux"

                PRETTY_NAME="Arch Linux"
                ID=arch
                ID_LIKE=archlinux
                VERSION_ID="foo"
                VERSION_CODENAME="bar"

            """
                )
            )
        )

        self.assertThat(release.id(), Equals("arch"))
        self.assertThat(release.name(), Equals("Arch Linux"))
        self.assertThat(release.version_id(), Equals("foo"))
        self.assertThat(release.version_codename(), Equals("bar"))

    def test_no_id(self):
        release = os_release.OsRelease(
            os_release_file=self._write_os_release(
                dedent(
                    """\
                NAME="Arch Linux"
                PRETTY_NAME="Arch Linux"
                ID_LIKE=archlinux
                VERSION_ID="foo"
                VERSION_CODENAME="bar"
            """
                )
            )
        )

        self.assertRaises(errors.OsReleaseIdError, release.id)

    def test_no_name(self):
        release = os_release.OsRelease(
            os_release_file=self._write_os_release(
                dedent(
                    """\
                ID=arch
                PRETTY_NAME="Arch Linux"
                ID_LIKE=archlinux
                VERSION_ID="foo"
                VERSION_CODENAME="bar"
            """
                )
            )
        )

        self.assertRaises(errors.OsReleaseNameError, release.name)

    def test_no_version_id(self):
        release = os_release.OsRelease(
            os_release_file=self._write_os_release(
                dedent(
                    """\
                NAME="Arch Linux"
                ID=arch
                PRETTY_NAME="Arch Linux"
                ID_LIKE=archlinux
                VERSION_CODENAME="bar"
            """
                )
            )
        )

        self.assertRaises(errors.OsReleaseVersionIdError, release.version_id)

    def test_no_version_codename(self):
        """Test that version codename can also come from VERSION_ID"""
        release = os_release.OsRelease(
            os_release_file=self._write_os_release(
                dedent(
                    """\
                NAME="Ubuntu"
                VERSION="14.04.5 LTS, Trusty Tahr"
                ID=ubuntu
                ID_LIKE=debian
                PRETTY_NAME="Ubuntu 14.04.5 LTS"
                VERSION_ID="14.04"
            """
                )
            )
        )

        self.assertThat(release.version_codename(), Equals("trusty"))

    def test_no_version_codename_or_version_id(self):
        release = os_release.OsRelease(
            os_release_file=self._write_os_release(
                dedent(
                    """\
                NAME="Ubuntu"
                ID=ubuntu
                ID_LIKE=debian
                PRETTY_NAME="Ubuntu 16.04.3 LTS"
            """
                )
            )
        )

        self.assertRaises(errors.OsReleaseCodenameError, release.version_codename)
