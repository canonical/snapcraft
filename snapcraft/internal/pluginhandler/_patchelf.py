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


def is_go_based_plugin(plugin):
    """Returns True if plugin is a go based one.

    :param plugin: the snapcraft plugin to inspect.
    :returns: True if plugin is one that uses go.
    :rtype: bool.
    """
    # We iterate over the plugins suppressing as they may not be loaded.
    plugin_exceptions = []
    with contextlib.suppress(AttributeError):
        plugin_exceptions.append(snapcraft.plugins.godeps.GodepsPlugin)
    with contextlib.suppress(AttributeError):
        plugin_exceptions.append(snapcraft.plugins.go.GoPlugin)
    return isinstance(plugin, tuple(plugin_exceptions))


class PartPatcher:
    """Takes care of patching files in a part if necessary."""

    def __init__(
        self,
        *,
        elf_files: FrozenSet[elf.ElfFile],
        plugin,
        project: ProjectOptions,
        confinement: str,  # TODO remove once project has this
        core_base: str,  # TODO remove once project has this
        snap_base_path: str,  # TODO remove once project has this
        stage_packages: List[str],
        stagedir: str,
        primedir: str
    ) -> None:
        """Initialize PartPatcher.

        :param elf_files: the list of elf files to analyze.
        :param plugin: the plugin the part is using.
        :param project: the project instance from the part.
        :param confinement: the confinement value the snapcraft project is
                            using (i.e.; devmode, strict, classic).
        :param core_base: the base the snap will use during runtime
                          (e.g.; core, core18).
        :param snap_base_path: the root path of the snap during runtime,
                               necessary when using a in-snap libc6 provided
                               as a stage-packages entry.
        :param stage_packages: the stage-packages a part is set to use.
        :param stagedir: the general stage directory for the snapcraft project.
                         This is used to locate an alternate patchelf binary
                         to use.
        :param primedir: the general prime directory for the snapcraft project.
        """
        self._elf_files = elf_files
        self._is_go_based_plugin = is_go_based_plugin(plugin)
        self._project = project
        self._is_classic = confinement == "classic"
        self._is_host_compat_with_base = project.is_host_compatible_with_base(core_base)
        self._core_base = core_base
        self._snap_base_path = snap_base_path
        # If libc6 is staged, to avoid symbol mixups we will resort to
        # glibc mangling.
        self._is_libc6_staged = "libc6" in stage_packages
        self._stagedir = stagedir
        self._primedir = primedir

    def _get_glibc_compatibility(self, linker_version: str) -> Dict[str, str]:
        linker_incompat = dict()  # type: Dict[str, str]
        for elf_file in self._elf_files:
            if not elf_file.is_linker_compatible(linker_version=linker_version):
                linker_incompat[elf_file.path] = elf_file.get_required_glibc()
        return linker_incompat

    def _get_preferred_patchelf_path(self):
        # TODO revisit if we need to support variations and permutations
        #  of this
        staged_patchelf_path = os.path.join(self._stagedir, "bin", "patchelf")
        if not os.path.exists(staged_patchelf_path):
            staged_patchelf_path = None
        return staged_patchelf_path

    def _patch(self, dynamic_linker: str) -> None:
        preferred_patchelf_path = self._get_preferred_patchelf_path()

        elf_patcher = elf.Patcher(
            dynamic_linker=dynamic_linker,
            root_path=self._primedir,
            preferred_patchelf_path=preferred_patchelf_path,
        )

        # Patching all files instead of a subset of them to ensure the
        # environment is consistent and the chain of dlopens that may
        # happen remains sane.
        for elf_file in self._elf_files:
            try:
                elf_patcher.patch(elf_file=elf_file)
            except errors.PatcherError as patch_error:
                logger.warning(
                    "An attempt to patch {!r} so that it would work "
                    "correctly in diverse environments was made and failed. "
                    "To disable this behavior set "
                    "`build-attributes: [no-patchelf]` for the part.".format(
                        elf_file.path
                    )
                )
                if not self._is_go_based_plugin:
                    raise patch_error

    def _verify_compat(self) -> None:
        linker_version = self._project._get_linker_version_for_base(self._core_base)
        linker_incompat = self._get_glibc_compatibility(linker_version)
        logger.debug(
            "List of files incompatible with {!r}: {!r}".format(
                linker_version, linker_incompat
            )
        )
        # Even though we do this in patch, it does not hurt to check again
        if linker_incompat and not self._is_libc6_staged:
            raise errors.IncompatibleBaseError(
                base=self._core_base,
                linker_version=linker_version,
                file_list=linker_incompat,
            )

    def patch(self) -> None:
        """Executes the patching process for elf_files.

        First it verifies there are no GLIBC mismatches, if any are found,
        libc6 must be part of stage-packages. The verification is only
        performed if the core base is not compatible with the host the
        snapcraft build is taking place on (e.g.; building on 18.04 while
        targeting 'core' as the core base).

        If the snapcraft project is not a classic confined one and there
        are no GLIBC mismatches, then this method returns with no modifications
        to elf_files. However, if one of those cases are true, the
        required dynamic linker is retrieved and the necessary elf files
        in elf_files are patched to behave accordinginly in the environment by
        means of setting the correct interpreter and rpaths (not runpaths).

        :raises errors.SnapcraftMissingLinkerInBaseError:
            if the linker within the core base cannot be found.
        :raises errors.StagePackageMissingError:
            if there are libc6 mismatches and libc6 is not in stage-packages.
        :raises errors.SnapcraftEnvironementError:
            if something is horribly wrong.
        """
        logger.debug(
            "Host compatible with base: {!r}".format(self._is_host_compat_with_base)
        )
        if not self._is_host_compat_with_base:
            self._verify_compat()
        logger.debug("Is classic: {!r}".format(self._is_classic))
        logger.debug("Is libc6 in stage-packages: {!r}".format(self._is_libc6_staged))
        if not (self._is_classic or self._is_libc6_staged):
            return

        if self._is_libc6_staged:
            dynamic_linker = elf.find_linker(
                root_path=self._primedir, snap_base_path=self._snap_base_path
            )
            logger.warning(
                "libc6 has been staged into the snap: only do this if you know what "
                "what you are doing."
            )
        elif self._is_classic:
            dynamic_linker = self._project.get_core_dynamic_linker(
                self._core_base, expand=False
            )
        else:
            raise errors.SnapcraftEnvironmentError(
                "An unexpected error has occurred while patching. "
                "Please log an issue against the snapcraft tool."
            )

        logger.debug("Dynamic linker set to {!r}".format(dynamic_linker))
        self._patch(dynamic_linker)
