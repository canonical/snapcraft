# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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

from snapcraft.internal import errors


_ARCH_TRANSLATIONS = {
    'armv7l': {
        'kernel': 'arm',
        'deb': 'armhf',
        'uts_machine': 'arm',
        'cross-compiler-prefix': 'arm-linux-gnueabihf-',
        'cross-build-packages': ['gcc-arm-linux-gnueabihf',
                                 'libc6-dev-armhf-cross'],
        'triplet': 'arm-linux-gnueabihf',
        'core-dynamic-linker': 'lib/ld-linux-armhf.so.3',
    },
    'aarch64': {
        'kernel': 'arm64',
        'deb': 'arm64',
        'uts_machine': 'aarch64',
        'cross-compiler-prefix': 'aarch64-linux-gnu-',
        'cross-build-packages': ['gcc-aarch64-linux-gnu',
                                 'libc6-dev-arm64-cross'],
        'triplet': 'aarch64-linux-gnu',
        'core-dynamic-linker': 'lib/ld-linux-aarch64.so.1',
    },
    'i686': {
        'kernel': 'x86',
        'deb': 'i386',
        'uts_machine': 'i686',
        'triplet': 'i386-linux-gnu',
    },
    'ppc64le': {
        'kernel': 'powerpc',
        'deb': 'ppc64el',
        'uts_machine': 'ppc64el',
        'cross-compiler-prefix': 'powerpc64le-linux-gnu-',
        'cross-build-packages': ['gcc-powerpc64le-linux-gnu',
                                 'libc6-dev-ppc64el-cross'],
        'triplet': 'powerpc64le-linux-gnu',
        'core-dynamic-linker': 'lib64/ld64.so.2',
    },
    'ppc': {
        'kernel': 'powerpc',
        'deb': 'powerpc',
        'uts_machine': 'powerpc',
        'cross-compiler-prefix': 'powerpc-linux-gnu-',
        'cross-build-packages': ['gcc-powerpc-linux-gnu',
                                 'libc6-dev-powerpc-cross'],
        'triplet': 'powerpc-linux-gnu',
    },
    'x86_64': {
        'kernel': 'x86',
        'deb': 'amd64',
        'uts_machine': 'x86_64',
        'triplet': 'x86_64-linux-gnu',
        'core-dynamic-linker': 'lib64/ld-linux-x86-64.so.2',
    },
    's390x': {
        'kernel': 's390',
        'deb': 's390x',
        'uts_machine': 's390x',
        'cross-compiler-prefix': 's390x-linux-gnu-',
        'cross-build-packages': ['gcc-s390x-linux-gnu',
                                 'libc6-dev-s390x-cross'],
        'triplet': 's390x-linux-gnu',
        'core-dynamic-linker': 'lib/ld64.so.1',
    }
}


_32BIT_USERSPACE_ARCHITECTURE = {
    'aarch64': 'armv7l',
    'armv8l': 'armv7l',
    'ppc64le': 'ppc',
    'x86_64': 'i686',
}


_WINDOWS_TRANSLATIONS = {
    'AMD64': 'x86_64'
}


_HOST_CODENAME_FOR_BASE = {
    'core18': 'bionic',
    'core': 'xenial',
}


_HOST_COMPATIBILITY = {
    'xenial': ['trusty', 'xenial'],
    'bionic': ['trusty', 'xenial', 'bionic'],
}


_LINKER_VERSION_FOR_BASE = {
    'core18': '2.27',
    'core': '2.23',
}


def _get_platform_architecture():
    architecture = platform.machine()

    # Translate the windows architectures we know of to architectures
    # we can work with.
    if sys.platform == 'win32':
        architecture = _WINDOWS_TRANSLATIONS.get(architecture)

    if platform.architecture()[0] == '32bit':
        userspace = _32BIT_USERSPACE_ARCHITECTURE.get(architecture)
        if userspace:
            architecture = userspace

    return architecture


def _get_deb_arch(machine):
    return _ARCH_TRANSLATIONS[machine].get('deb', None)


def _find_machine(deb_arch):
    for machine in _ARCH_TRANSLATIONS:
        if _ARCH_TRANSLATIONS[machine].get('deb', '') == deb_arch:
            return machine
        elif _ARCH_TRANSLATIONS[machine].get('uts_machine', '') == deb_arch:
            return machine

    raise errors.SnapcraftEnvironmentError(
        'Cannot set machine from deb_arch {!r}'.format(deb_arch))
