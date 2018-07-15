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

import contextlib


_ID_TO_UBUNTU_CODENAME = {
    "17.10": "artful",
    "17.04": "zesty",
    "16.04": "xenial",
    "14.04": "trusty",
}


def get_version_codename() -> str:
    """Return the OS version codename

    This first tries to use the VERSION_CODENAME. If that's missing, it
    tries to use the VERSION_ID to figure out the codename on its own.

    """
    os_release = {}  # type: Dict[str, str]
    with contextlib.suppress(FileNotFoundError):
        with open("/etc/os-release") as f:
            for line in f:
                entry = line.rstrip().split("=")
                if len(entry) == 2:
                    os_release[entry[0]] = entry[1].strip('"')

    with contextlib.suppress(KeyError):
        return os_release["VERSION_CODENAME"]

    with contextlib.suppress(KeyError):
        return _ID_TO_UBUNTU_CODENAME[os_release["VERSION_ID"]]

    return None
