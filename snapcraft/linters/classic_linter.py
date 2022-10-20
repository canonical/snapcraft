# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

"""Classic linter implementation."""

from pathlib import Path
from typing import List

from overrides import overrides

from snapcraft.elf import ElfFile, Patcher, SonameCache, elf_utils, errors

from .base import Linter, LinterIssue, LinterResult

_HELP_URL = "https://forum.snapcraft.io/t/classic-linter/32228"


class ClassicLinter(Linter):
    """Linter for classic snaps."""

    @overrides
    def run(self) -> List[LinterIssue]:
        if not self._snap_metadata.base or self._snap_metadata.base == "bare":
            return []

        current_path = Path()
        installed_snap_path = Path(f"/snap/{self._snap_metadata.name}/current")
        installed_base_path = Path(f"/snap/{self._snap_metadata.base}/current")

        # ELF files must be patched if confinement is classic or libc is staged
        try:
            # If libc is staged we'll find a dynamic linker in the current path
            # (which contains the unpackaged primed payload). In this case, at
            # runtime the linker will be in the installed snap path.
            linker = elf_utils.get_dynamic_linker(
                root_path=current_path, snap_path=installed_snap_path
            )
            is_libc_staged = True
            issue = LinterIssue(
                name=self._name,
                result=LinterResult.OK,
                text="Snap contains staged libc.",
            )
        except errors.DynamicLinkerNotFound:
            # Otherwise look for the host linker, which should match the base
            # system linker. At runtime use the linker from the installed base
            # snap.
            linker = elf_utils.get_dynamic_linker(
                root_path=Path("/"), snap_path=installed_base_path
            )
            is_libc_staged = False
            issue = LinterIssue(
                name=self._name,
                result=LinterResult.OK,
                text="Snap confinement is set to classic.",
            )

        # No issues to report if confinement is not classic and libc is not staged.
        if not is_libc_staged and self._snap_metadata.confinement != "classic":
            return []

        issues = [issue]
        elf_files = elf_utils.get_elf_files(current_path)
        patcher = Patcher(dynamic_linker=linker, root_path=current_path.absolute())
        soname_cache = SonameCache()

        for elf_file in elf_files:
            # Skip linting files listed in the ignore list.
            if self._is_file_ignored(elf_file):
                continue

            arch_triplet = elf_utils.get_arch_triplet()

            elf_file.load_dependencies(
                root_path=current_path.absolute(),
                base_path=installed_base_path,
                content_dirs=self._snap_metadata.get_provider_content_directories(),
                arch_triplet=arch_triplet,
                soname_cache=soname_cache,
            )

            self._check_elf_interpreter(elf_file, linker=linker, issues=issues)
            self._check_elf_rpath(elf_file, patcher=patcher, issues=issues)

        return issues

    def _check_elf_interpreter(
        self, elf_file: ElfFile, *, linker: str, issues: List[LinterIssue]
    ) -> None:
        """Check ELF executable interpreter is set to base or snap linker."""
        if elf_file.interp != linker:
            issue = LinterIssue(
                name=self._name,
                result=LinterResult.WARNING,
                filename=str(elf_file.path),
                text=f"ELF interpreter should be set to {linker!r}.",
                url=_HELP_URL,
            )
            issues.append(issue)

    def _check_elf_rpath(
        self,
        elf_file: ElfFile,
        *,
        patcher: Patcher,
        issues: List[LinterIssue],
    ) -> None:
        """Check if the ELF executable rpath points to base or current snap."""
        current_rpath = patcher.get_current_rpath(elf_file)
        proposed_rpath = patcher.get_proposed_rpath(elf_file)

        if not set(proposed_rpath).issubset(set(current_rpath)):
            formatted_rpath = ":".join(proposed_rpath)
            issue = LinterIssue(
                name=self._name,
                result=LinterResult.WARNING,
                filename=str(elf_file.path),
                text=f"ELF rpath should be set to {formatted_rpath!r}.",
                url=_HELP_URL,
            )
            issues.append(issue)
