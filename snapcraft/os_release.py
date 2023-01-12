# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2017-2022 Canonical Ltd.
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

"""OS release information helpers."""

import contextlib
from pathlib import Path
from typing import Dict

from snapcraft import errors

_ID_TO_UBUNTU_CODENAME = {
    "17.10": "artful",
    "17.04": "zesty",
    "16.04": "xenial",
    "14.04": "trusty",
}


class OsRelease:
    """A class to intelligently determine the OS on which we're running."""

    def __init__(
        self,
        *,
        os_release_file: Path = Path(  # noqa: B008 Function call in arg defaults
            "/etc/os-release"
        )
    ) -> None:
        """Create a new OsRelease instance.

        :param str os_release_file: Path to os-release file to be parsed.
        """
        with contextlib.suppress(FileNotFoundError):
            self._os_release: Dict[str, str] = {}
            with os_release_file.open(encoding="utf-8") as release_file:
                for line in release_file:
                    entry = line.rstrip().split("=")
                    if len(entry) == 2:
                        self._os_release[entry[0]] = entry[1].strip('"')

    def id(self) -> str:
        """Return the OS ID.

        :raises SnapcraftError: If no ID can be determined.
        """
        with contextlib.suppress(KeyError):
            return self._os_release["ID"]

        raise errors.SnapcraftError("Unable to determine host OS ID")

    def name(self) -> str:
        """Return the OS name.

        :raises SnapcraftError: If no name can be determined.
        """
        with contextlib.suppress(KeyError):
            return self._os_release["NAME"]

        raise errors.SnapcraftError("Unable to determine host OS name")

    def version_id(self) -> str:
        """Return the OS version ID.

        :raises SnapcraftError: If no version ID can be determined.
        """
        with contextlib.suppress(KeyError):
            return self._os_release["VERSION_ID"]

        raise errors.SnapcraftError("Unable to determine host OS version ID")

    def version_codename(self) -> str:
        """Return the OS version codename.

        This first tries to use the VERSION_CODENAME. If that's missing, it
        tries to use the VERSION_ID to figure out the codename on its own.

        :raises SnapcraftError: If no version codename can be determined.
        """
        with contextlib.suppress(KeyError):
            return self._os_release["VERSION_CODENAME"]

        with contextlib.suppress(KeyError):
            return _ID_TO_UBUNTU_CODENAME[self._os_release["VERSION_ID"]]

        raise errors.SnapcraftError("Unable to determine host OS version codename")
