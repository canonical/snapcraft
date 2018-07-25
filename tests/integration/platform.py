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

import platform
import sys


_32BIT_USERSPACE_ARCHITECTURE = {
    "aarch64": "armv7l",
    "armv8l": "armv7l",
    "ppc64le": "ppc",
    "x86_64": "i686",
}


_ARCH_TRANSLATIONS = {
    "armv7l": {"deb": "armhf", "triplet": "arm-linux-gnueabihf"},
    "aarch64": {"deb": "arm64", "triplet": "aarch64-linux-gnu"},
    "i686": {"deb": "i386", "triplet": "i386-linux-gnu"},
    "ppc64le": {"deb": "ppc64el", "triplet": "powerpc64le-linux-gnu"},
    "ppc": {"deb": "powerpc", "triplet": "powerpc-linux-gnu"},
    "x86_64": {"deb": "amd64", "triplet": "x86_64-linux-gnu"},
    "s390x": {"deb": "s390x", "triplet": "s390x-linux-gnu"},
}


_WINDOWS_TRANSLATIONS = {"AMD64": "x86_64"}


def get_deb_arch():
    return _ARCH_TRANSLATIONS[_get_platform_architecture()]["deb"]


def get_arch_triplet():
    return _ARCH_TRANSLATIONS[_get_platform_architecture()]["triplet"]


def _get_platform_architecture():
    architecture = platform.machine()

    # Translate the windows architectures we know of to architectures
    # we can work with.
    if sys.platform == "win32":
        architecture = _WINDOWS_TRANSLATIONS.get(architecture)

    if platform.architecture()[0] == "32bit":
        userspace = _32BIT_USERSPACE_ARCHITECTURE.get(architecture)
        if userspace:
            architecture = userspace

    return architecture
