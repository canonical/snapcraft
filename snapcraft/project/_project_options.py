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

import logging
import multiprocessing
import os
from contextlib import suppress
from typing import List, Set, Union  # noqa: F401

from snapcraft import file_utils
from snapcraft.internal import common, errors, os_release
from snapcraft.internal.deprecations import handle_deprecation_notice


from ._arch_mapping import (
    _ARCH_TRANSLATIONS,
    _HOST_CODENAME_FOR_BASE,
    _HOST_COMPATIBILITY,
    _LINKER_VERSION_FOR_BASE,
    _get_platform_architecture,
    _find_machine
)


logger = logging.getLogger(__name__)


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
        return self.__target_machine != self.__platform_arch

    @property
    def target_arch(self):
        return self.__target_arch

    @property
    def cross_compiler_prefix(self):
        try:
            # cross-compilation of x86 32bit binaries on a x86_64 host is
            # possible by reusing the native toolchain - let Kbuild figure
            # it out by itself and pass down an empy cross-compiler-prefix
            # to start the build
            if (self.__platform_arch == 'x86_64' and
                    self.__target_machine == 'i686'):
                return ''
            return self.__machine_info['cross-compiler-prefix']
        except KeyError:
            raise errors.SnapcraftEnvironmentError(
                'Cross compilation not supported for target arch {!r}'.format(
                    self.__target_machine))

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
        deprecated_plugins_dir = os.path.join(self.parts_dir, 'plugins')
        if os.path.exists(deprecated_plugins_dir):
            handle_deprecation_notice('dn2')
            return deprecated_plugins_dir
        return os.path.join(self.__project_dir, 'snap', 'plugins')

    @property
    def parts_dir(self):
        return os.path.join(self.__project_dir, 'parts')

    @property
    def stage_dir(self):
        return os.path.join(self.__project_dir, 'stage')

    @property
    def prime_dir(self):
        return os.path.join(self.__project_dir, 'prime')

    @property
    def debug(self):
        return self.__debug

    def __init__(self, use_geoip=False, parallel_builds=True,
                 target_deb_arch=None, debug=False):
        # TODO: allow setting a different project dir and check for
        #       snapcraft.yaml
        self.__project_dir = os.getcwd()
        self.__use_geoip = use_geoip
        self.__parallel_builds = parallel_builds
        self._set_machine(target_deb_arch)
        self.__debug = debug

    def is_host_compatible_with_base(self, base: str) -> bool:
        """Determines if the host is compatible with the GLIBC of the base.

        The system should warn early on when building using a host that does
        not match the intended base, this mechanism here enables additional
        logic when that is ignored to determine built projects will actually
        run.

        :param str base: the base core snap to search for linker.
        :returns: True if there are no GLIBC incompatibilities with the chosen
                  build host, else it returns False.
        :rtype: bool
        """
        codename = None  # type: str
        with suppress(errors.OsReleaseCodenameError):
            codename = os_release.OsRelease().version_codename()
            logger.debug('Running on {!r}'.format(codename))

        build_host_for_base = _HOST_CODENAME_FOR_BASE.get(
            base)  # type: str
        compatible_hosts = _HOST_COMPATIBILITY.get(
            build_host_for_base, [])  # type: List[str]
        return codename in compatible_hosts

    # This is private to not make the API public given that base
    # will be part of the new Project.
    def _get_linker_version_for_base(self, base: str) -> str:
        """Returns the linker version for base."""
        try:
            return _LINKER_VERSION_FOR_BASE[base]
        except KeyError:
            linker_file = os.path.basename(self.get_core_dynamic_linker(base))
            return file_utils.get_linker_version_from_file(linker_file)

    def get_core_dynamic_linker(self, base: str, expand: bool=True) -> str:
        """Returns the dynamic linker used for the targeted core.

        :param str base: the base core snap to search for linker.
        :param bool expand: expand the linker to the actual linker if True,
                            else the main entry point to the linker for the
                            projects architecture.
        :return: the absolute path to the linker
        :rtype: str
        :raises snapcraft.internal.errors.SnapcraftMissingLinkerInBaseError:
            if the linker cannot be found in the base.
        :raises snapcraft.internal.errors.SnapcraftEnvironmentError:
            if a loop is found while resolving the real path to the linker.
        """
        core_path = common.get_core_path(base)
        dynamic_linker_path = os.path.join(
            core_path,
            self.__machine_info.get('core-dynamic-linker',
                                    'lib/ld-linux.so.2'))

        # return immediately if we do not need to expand
        if not expand:
            return dynamic_linker_path

        # We can't use os.path.realpath because any absolute symlinks
        # have to be interpreted relative to core_path, not the real
        # root.
        seen_paths = set()  # type: Set[str]
        while True:
            if dynamic_linker_path in seen_paths:
                raise errors.SnapcraftEnvironmentError(
                    "found symlink loop resolving dynamic linker path")

            seen_paths.add(dynamic_linker_path)
            if not os.path.lexists(dynamic_linker_path):
                raise errors.SnapcraftMissingLinkerInBaseError(
                    base=base, linker_path=dynamic_linker_path)
            if not os.path.islink(dynamic_linker_path):
                return dynamic_linker_path

            link_contents = os.readlink(dynamic_linker_path)
            if os.path.isabs(link_contents):
                dynamic_linker_path = os.path.join(
                    core_path, link_contents.lstrip('/'))
            else:
                dynamic_linker_path = os.path.join(
                    os.path.dirname(dynamic_linker_path), link_contents)

    def _set_machine(self, target_deb_arch):
        self.__platform_arch = _get_platform_architecture()
        self.__target_arch = target_deb_arch
        if not target_deb_arch:
            self.__target_machine = self.__platform_arch
        else:
            self.__target_machine = _find_machine(target_deb_arch)
            logger.info('Setting target machine to {!r}'.format(
                target_deb_arch))
        self.__machine_info = _ARCH_TRANSLATIONS[self.__target_machine]
