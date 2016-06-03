# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

import logging
import multiprocessing
import os
import platform


logger = logging.getLogger(__name__)


_ARCH_TRANSLATIONS = {
    'armv7l': {
        'kernel': 'arm',
        'deb': 'armhf',
        'cross-compiler-prefix': 'arm-linux-gnueabihf-',
        'cross-build-packages': ['gcc-arm-linux-gnueabihf'],
        'triplet': 'arm-linux-gnueabihf',
    },
    'aarch64': {
        'kernel': 'arm64',
        'deb': 'arm64',
        'cross-compiler-prefix': 'aarch64-linux-gnu-',
        'cross-build-packages': ['gcc-aarch64-linux-gnu'],
        'triplet': 'aarch64-linux-gnu',
    },
    'i686': {
        'kernel': 'x86',
        'deb': 'i386',
        'triplet': 'i386-linux-gnu',
    },
    'ppc64le': {
        'kernel': 'powerpc',
        'deb': 'ppc64el',
        'cross-compiler-prefix': 'powerpc64le-linux-gnu-',
        'cross-build-packages': ['gcc-powerpc64le-linux-gnu'],
        'triplet': 'powerpc64le-linux-gnu',
    },
    'x86_64': {
        'kernel': 'x86',
        'deb': 'amd64',
        'triplet': 'x86_64-linux-gnu',
    },
    's390x': {
        'kernel': 's390x',
        'deb': 's390x',
        'cross-compiler-prefix': 's390x-linux-gnu-',
        'cross-build-packages': ['gcc-s390x-linux-gnu'],
        'triplet': 's390x-linux-gnu',
    }
}


class ProjectOptions:

    @property
    def use_geoip(self):
        return self.__use_geoip

    @property
    def parallel_builds(self):
        return self.__parallel_builds

    @property
    def parallel_build_count(self):
        build_count = 1
        if self.__parallel_builds:
            try:
                build_count = multiprocessing.cpu_count()
            except NotImplementedError:
                logger.warning(
                    'Unable to determine CPU count; disabling parallel builds')

        return build_count

    @property
    def is_cross_compiling(self):
        return self.__target_machine != self.__host_machine

    @property
    def cross_compiler_prefix(self):
        try:
            return self.__machine_info['cross-compiler-prefix']
        except KeyError:
            raise EnvironmentError(
                'Cross compilation not support for target arch {!}'.format(
                    self.__machine_target))

    @property
    def additional_build_packages(self):
        packages = []
        if self.is_cross_compiling:
            packages.extend(self.__machine_info.get(
                'cross-build-packages', []))
        return packages

    @property
    def arch_triplet(self):
        return self.__machine_info['triplet']

    @property
    def deb_arch(self):
        return self.__machine_info['deb']

    @property
    def kernel_arch(self):
        return self.__machine_info['kernel']

    @property
    def local_plugins_dir(self):
        return os.path.join(self.parts_dir, 'plugins')

    @property
    def parts_dir(self):
        return os.path.join(self.__project_dir, 'parts')

    @property
    def stage_dir(self):
        return os.path.join(self.__project_dir, 'stage')

    @property
    def snap_dir(self):
        return os.path.join(self.__project_dir, 'prime')

    def __init__(self, use_geoip=False, parallel_builds=True,
                 target_deb_arch=None):
        # TODO: allow setting a different project dir and check for
        #       snapcraft.yaml
        self.__project_dir = os.getcwd()
        self.__use_geoip = use_geoip
        self.__parallel_builds = parallel_builds
        self._set_machine(target_deb_arch)

    def _set_machine(self, target_deb_arch):
        self.__host_machine = platform.machine()
        if not target_deb_arch:
            self.__target_machine = self.__host_machine
        else:
            self.__target_machine = _find_machine(target_deb_arch)
            logger.info('Setting target machine to {!r}'.format(
                target_deb_arch))
        self.__machine_info = _ARCH_TRANSLATIONS[self.__target_machine]


def _find_machine(deb_arch):
    for machine in _ARCH_TRANSLATIONS:
        if _ARCH_TRANSLATIONS[machine].get('deb', '') == deb_arch:
            return machine

    raise EnvironmentError(
        'Cannot set machine from deb_arch {!r}'.format(deb_arch))
