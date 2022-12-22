# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2022 Canonical Ltd.
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

"""Use patchelf to patch ELF files."""

import functools
import os
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import List, Set

from craft_cli import emit

from snapcraft import utils

from ._elf_file import ElfFile
from .errors import PatcherError


class Patcher:
    """Hold the necessary logic to patch elf files."""

    def __init__(
        self, *, dynamic_linker: str, root_path: Path, preferred_patchelf=None
    ) -> None:
        """Create a Patcher instance.

        :param dynamic_linker: The path to the dynamic linker to set the ELF file to.
        :param root_path: The base path for the snap being processed.
        :param preferred_patchelf: patch the necessary elf_files with this patchelf.
        """
        self._dynamic_linker = dynamic_linker
        self._root_path = root_path
        self._patchelf_cmd = preferred_patchelf or utils.get_snap_tool("patchelf")
        self._strip_cmd = utils.get_snap_tool("strip")

    def patch(self, *, elf_file: ElfFile) -> None:
        """Patch elf_file with the Patcher instance configuration.

        If the ELF is executable, patch it to use the configured linker.
        If the ELF has dependencies (DT_NEEDED), set an rpath to them.

        :param elf_file: a data object representing an elf file and its attributes.

        :raises PatcherError: if the ELF file cannot be patched.
        """
        patchelf_args = []
        if elf_file.interp and elf_file.interp != self._dynamic_linker:
            patchelf_args.extend(["--set-interpreter", self._dynamic_linker])
            emit.progress(f"  Interpreter={self._dynamic_linker!r}")

        if elf_file.dependencies:
            current_rpath = self.get_current_rpath(elf_file)
            proposed_rpath = self.get_proposed_rpath(elf_file)

            emit.progress(f"  Current rpath={current_rpath}")
            emit.progress(f"  Proposed rpath={proposed_rpath}")

            # Removing the current rpath should not be necessary after patchelf 0.11,
            # see https://github.com/NixOS/patchelf/issues/94

            # Parameters:
            # --force-rpath: use RPATH instead of RUNPATH.
            # --shrink-rpath: will remove unneeded entries, with the side effect of
            #                 preferring host libraries so we simply do not use it.
            # --set-rpath: set the RPATH to the colon separated argument.

            # Don't need to patch the binary if all proposed paths are already in rpath
            if not set(proposed_rpath).issubset(set(current_rpath)):
                formatted_rpath = ":".join(proposed_rpath)
                patchelf_args.extend(["--force-rpath", "--set-rpath", formatted_rpath])

        # no patchelf_args means there is nothing to do.
        if not patchelf_args:
            return

        self._run_patchelf(patchelf_args=patchelf_args, elf_file_path=elf_file.path)

    def _run_patchelf(self, *, patchelf_args: List[str], elf_file_path: Path) -> None:
        # Run patchelf on a copy of the primed file and replace it
        # after it is successful. This allows us to break the potential
        # hard link created when migrating the file across the steps of
        # the part.
        with tempfile.NamedTemporaryFile() as temp_file:
            shutil.copy2(elf_file_path, temp_file.name)

            cmd = [self._patchelf_cmd] + patchelf_args + [temp_file.name]
            try:
                emit.debug(f"executing: {' '.join(cmd)}")
                subprocess.check_call(cmd)
            # There is no need to catch FileNotFoundError as patchelf should be
            # bundled with snapcraft which means its lack of existence is a
            # "packager" error.
            except subprocess.CalledProcessError as call_error:
                raise PatcherError(
                    elf_file_path, cmd=call_error.cmd, code=call_error.returncode
                ) from call_error

            # We unlink to break the potential hard link
            os.unlink(elf_file_path)
            shutil.copy2(temp_file.name, elf_file_path)

    @functools.lru_cache(maxsize=1024)
    def get_current_rpath(self, elf_file: ElfFile) -> List[str]:
        """Obtain the current rpath from the ELF file dynamic section."""
        output = subprocess.check_output(
            [self._patchelf_cmd, "--print-rpath", str(elf_file.path)]
        )
        return [x for x in output.decode().strip().split(":") if x]

    @functools.lru_cache(maxsize=1024)
    def get_proposed_rpath(self, elf_file: ElfFile) -> List[str]:
        """Obtain the proposed rpath pointing to the base or application snaps."""
        origin_rpaths: List[str] = []
        base_rpaths: Set[str] = set()
        existing_rpaths = self.get_current_rpath(elf_file)

        for dependency in elf_file.dependencies:
            if dependency.path:
                if dependency.in_base_snap:
                    base_rpaths.add(os.path.dirname(dependency.path))
                elif self._root_path in dependency.path.parents:
                    rel_library_path = os.path.relpath(dependency.path, elf_file.path)
                    rel_library_path_dir = os.path.dirname(rel_library_path)
                    # return the dirname, with the first .. replace
                    # with $ORIGIN
                    origin_rpath = rel_library_path_dir.replace("..", "$ORIGIN", 1)
                    if origin_rpath not in origin_rpaths:
                        origin_rpaths.append(origin_rpath)

        if existing_rpaths:
            # Only keep those that mention origin and are not already in our bundle.
            existing_rpaths = [
                r for r in existing_rpaths if "$ORIGIN" in r and r not in origin_rpaths
            ]
            origin_rpaths = existing_rpaths + origin_rpaths

        origin_rpath_list = [r for r in origin_rpaths if r]
        base_rpath_list = sorted(base_rpaths)

        return origin_rpath_list + base_rpath_list
