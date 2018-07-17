# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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
from unittest import mock

import fixtures

from snapcraft.internal import os_release


class FakeOsRelease(fixtures.Fixture):
    def __init__(
        self,
        id: str = "ubuntu",
        version_id: str = "16.04",
        version_codename: str = None,
    ) -> None:
        self._id = id
        self._version_id = version_id
        self._version_codename = version_codename

    def _setUp(self):
        super()._setUp()

        with open("os-release", "w") as release_file:
            print(
                dedent(
                    """\
                NAME="Ubuntu"
                VERSION="16.04.3 LTS (Xenial Xerus)"
                ID_LIKE=debian
                PRETTY_NAME="Ubuntu 16.04.3 LTS"
                HOME_URL="http://www.ubuntu.com/"
                SUPPORT_URL="http://help.ubuntu.com/"
                BUG_REPORT_URL="http://bugs.launchpad.net/ubuntu/"
                UBUNTU_CODENAME=xenial"""
                ),
                file=release_file,
            )
            if self._id is not None:
                print("ID={}".format(self._id), file=release_file)
            if self._version_id is not None:
                print('VERSION_ID="{}"'.format(self._version_id), file=release_file)
            if self._version_codename is not None:
                print(
                    "VERSION_CODENAME={}".format(self._version_codename),
                    file=release_file,
                )

        release = os_release.OsRelease(os_release_file="os-release")

        def _create_os_release(*args, **kwargs):
            return release

        patcher = mock.patch(
            "snapcraft.internal.os_release.OsRelease", wraps=_create_os_release
        )
        patcher.start()
        self.addCleanup(patcher.stop)
