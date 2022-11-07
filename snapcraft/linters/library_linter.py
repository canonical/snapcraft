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

"""Library linter implementation."""

from pathlib import Path, PurePath
from typing import List

from overrides import overrides

from snapcraft.elf import ElfFile, SonameCache, elf_utils

from .base import Linter, LinterIssue, LinterResult


class LibraryLinter(Linter):
    """Linter for dynamic library availability in snap."""

    @overrides
    def run(self) -> List[LinterIssue]:
        if self._snap_metadata.type not in ("app", None):
            return []

        current_path = Path()
        if self._snap_metadata.base and self._snap_metadata.base != "bare":
            installed_base_path = Path(f"/snap/{self._snap_metadata.base}/current")
        else:
            installed_base_path = None

        issues: List[LinterIssue] = []
        elf_files = elf_utils.get_elf_files(current_path)
        soname_cache = SonameCache()

        for elf_file in elf_files:
            # Skip linting files listed in the ignore list.
            if self._is_file_ignored(elf_file):
                continue

            arch_triplet = elf_utils.get_arch_triplet()
            content_dirs = self._snap_metadata.get_provider_content_directories()

            dependencies = elf_file.load_dependencies(
                root_path=current_path.absolute(),
                base_path=installed_base_path,
                content_dirs=content_dirs,
                arch_triplet=arch_triplet,
                soname_cache=soname_cache,
            )

            search_paths = [current_path.absolute(), *content_dirs]
            if installed_base_path:
                search_paths.append(installed_base_path)

            self._check_dependencies_satisfied(
                elf_file,
                search_paths=search_paths,
                dependencies=sorted(dependencies),
                issues=issues,
            )

        return issues

    def _check_dependencies_satisfied(
        self,
        elf_file: ElfFile,
        *,
        search_paths: List[Path],
        dependencies: List[str],
        issues: List[LinterIssue],
    ) -> None:
        """Check if ELF executable dependencies are satisfied by snap files.

        :param elf_file: The ELF file whose dependencies are being verified.
        :param root_path: The absolute path to the payload directory.
        :param issues: The list of linter issues.
        """
        for dependency in dependencies:
            dependency_path = PurePath(dependency)

            for path in search_paths:
                if path in dependency_path.parents:
                    break
            else:
                issue = LinterIssue(
                    name=self._name,
                    result=LinterResult.WARNING,
                    filename=str(elf_file.path),
                    text=f"missing dependency {dependency_path.name!r}.",
                    url="https://snapcraft.io/docs/linters-library",
                )
                issues.append(issue)
