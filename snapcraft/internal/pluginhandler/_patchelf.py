# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2018 Canonical Ltd
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
import logging
import os
from typing import FrozenSet, List
from typing import Dict  # noqa: F401

import snapcraft.plugins
from snapcraft import ProjectOptions
from snapcraft.internal import elf
from snapcraft.internal import errors


logger = logging.getLogger(__name__)


def _is_go_based_plugin(plugin):
    # We iterate over the plugins suppressing as they may not be loaded.
    plugin_exceptions = []
    with contextlib.suppress(AttributeError):
        plugin_exceptions.append(snapcraft.plugins.godeps.GodepsPlugin)
    with contextlib.suppress(AttributeError):
        plugin_exceptions.append(snapcraft.plugins.go.GoPlugin)
    return isinstance(plugin, tuple(plugin_exceptions))


class PartPatcher:

    def __init__(self, *,
                 elf_files: FrozenSet[elf.ElfFile],
                 plugin,
                 project: ProjectOptions,
                 confinement: str,     # TODO remove once project has this
                 core_base: str,       # TODO remove once project has this
                 snap_base_path: str,  # TODO remove once project has this
                 stage_packages: List[str],
                 stagedir: str,
                 primedir: str) -> None:
        self._elf_files = elf_files
        self._is_go_based_plugin = _is_go_based_plugin(plugin)
        self._project = project
        self._is_classic = confinement == 'classic'
        self._is_host_compat_with_base = project.is_host_compatible_with_base(
            core_base)
        self._core_base = core_base
        self._snap_base_path = snap_base_path
        # If libc6 is staged, to avoid symbol mixups we will resort to
        # glibc mangling.
        self._is_libc6_staged = 'libc6' in stage_packages
        self._stagedir = stagedir
        self._primedir = primedir

    def _get_glibc_compatibility(self, linker_version: str) -> Dict[str, str]:
        linker_incompat = dict()  # type: Dict[str, str]
        for elf_file in self._elf_files:
            if not elf_file.is_linker_compatible(linker=linker_version):
                linker_incompat[elf_file.path] = \
                    elf_file.get_required_glibc()
        if linker_incompat:
            formatted_items = ['- {} (requires GLIBC {})'.format(k, v)
                               for k, v in linker_incompat.items()]
            logger.warning(
                'The GLIBC version of the targeted core is {}. A newer '
                'libc will be required for the following files:'
                '\n{}'.format(linker_version, '\n'.join(formatted_items)))
        return linker_incompat

    def _get_preferred_patchelf_path(self):
        # TODO revisit if we need to support variations and permutations
        #  of this
        staged_patchelf_path = os.path.join(self._stagedir, 'bin', 'patchelf')
        if not os.path.exists(staged_patchelf_path):
            staged_patchelf_path = None
        return staged_patchelf_path

    def _patch(self, dynamic_linker: str) -> None:
        preferred_patchelf_path = self._get_preferred_patchelf_path()

        elf_patcher = elf.Patcher(
            dynamic_linker=dynamic_linker,
            root_path=self._primedir,
            preferred_patchelf_path=preferred_patchelf_path)
        files_to_patch = elf.get_elf_files_to_patch(self._elf_files)
        for elf_file in files_to_patch:
            try:
                elf_patcher.patch(elf_file=elf_file)
            except errors.PatcherError as patch_error:
                logger.warning(
                    'An attempt to patch {!r} so that it would work '
                    'correctly in diverse environments was made and failed. '
                    'To disable this behavior set '
                    '`build-attributes: [no-patchelf]` for the part.')
                if not self._is_go_based_plugin:
                    raise patch_error

    def _verify_compat(self) -> None:
        linker_version = os.path.basename(
                self._project.get_core_dynamic_linker(self._core_base))
        linker_incompat = self._get_glibc_compatibility(linker_version)
        # Even though we do this in patch, it does not hurt to check again
        logger.debug('Is the {!r} incompatible: {!r}'.format(linker_version,
                                                             linker_incompat))
        if linker_incompat and not self._is_libc6_staged:
            raise errors.StagePackageMissingError(package='libc6')

    def patch(self) -> None:
        # Rules for verification
        #   the host is not compatible with the selected base
        logger.debug('Host compatible with base: {!r}'.format(
            self._is_host_compat_with_base))
        if not self._is_host_compat_with_base:
            self._verify_compat()
        # Rules for patching:
        #   if the confinement is classic
        #   if libc6 is staged
        logger.debug('Is classic: {!r}'.format(self._is_classic))
        logger.debug('Is libc6 in stage-packages: {!r}'.format(
            self._is_libc6_staged))
        if not (self._is_classic or self._is_libc6_staged):
            return

        if self._is_libc6_staged:
            dynamic_linker = elf.find_linker(
                root_path=self._primedir,
                snap_base_path=self._snap_base_path)
        elif self._is_classic:
            dynamic_linker = self._project.get_core_dynamic_linker(
                self._core_base, expand=False)
        else:
            raise errors.SnapcraftEnvironmentError(
                'An unexpected error has occured while patching. '
                'Please log an issue against the snapcraft tool.')

        logger.debug('Dynamic linker set to {!r}'.format(dynamic_linker))
        self._patch(dynamic_linker)
