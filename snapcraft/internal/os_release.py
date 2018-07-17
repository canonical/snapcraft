# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import contextlib

# This is used in a mypy 'type:' comment below, but nowhere else. Flake8
# doesn't like that very much, so noqa.
from typing import Dict  # noqa

from snapcraft.internal import errors

_ID_TO_UBUNTU_CODENAME = {
    "17.10": "artful",
    "17.04": "zesty",
    "16.04": "xenial",
    "14.04": "trusty",
}


class OsRelease:
    """A class to intelligently determine the OS on which we're running"""

    def __init__(self, *, os_release_file: str = "/etc/os-release") -> None:
        """Create a new OsRelease instance.

        :param str os_release_file: Path to os-release file to be parsed.
        """
        with contextlib.suppress(FileNotFoundError):
            self._os_release = {}  # type: Dict[str, str]
            with open(os_release_file) as f:
                for line in f:
                    entry = line.rstrip().split("=")
                    if len(entry) == 2:
                        self._os_release[entry[0]] = entry[1].strip('"')

    def id(self) -> str:
        """Return the OS ID

        :raises OsReleaseIdError: If no ID can be determined.
        """
        with contextlib.suppress(KeyError):
            return self._os_release["ID"]

        raise errors.OsReleaseIdError()

    def name(self) -> str:
        """Return the OS name

        :raises OsReleaseNameError: If no name can be determined.
        """
        with contextlib.suppress(KeyError):
            return self._os_release["NAME"]

        raise errors.OsReleaseNameError()

    def version_id(self) -> str:
        """Return the OS version ID

        :raises OsReleaseVersionIdError: If no version ID can be determined.
        """
        with contextlib.suppress(KeyError):
            return self._os_release["VERSION_ID"]

        raise errors.OsReleaseVersionIdError()

    def version_codename(self) -> str:
        """Return the OS version codename

        This first tries to use the VERSION_CODENAME. If that's missing, it
        tries to use the VERSION_ID to figure out the codename on its own.

        :raises OsReleaseCodenameError: If no version codename can be
                                        determined.
        """
        with contextlib.suppress(KeyError):
            return self._os_release["VERSION_CODENAME"]

        with contextlib.suppress(KeyError):
            return _ID_TO_UBUNTU_CODENAME[self._os_release["VERSION_ID"]]

        raise errors.OsReleaseCodenameError()
