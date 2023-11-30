# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022-2023 Canonical Ltd.
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
import re
import subprocess
from pathlib import Path
from typing import List, Set

from craft_cli import emit
from overrides import overrides

from snapcraft.elf import ElfFile, SonameCache, elf_utils
from snapcraft.elf import errors as elf_errors

from .base import Linter, LinterIssue, LinterResult, Optional


class LibraryLinter(Linter):
    """Linter for dynamic library availability in snap."""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._ld_config_cache: dict[str, Path] = {}

    @staticmethod
    def get_categories() -> List[str]:
        """Get the specific sub-categories that can be filtered against."""
        return ["unused-library", "missing-library"]

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
        all_libraries: Set[Path] = set()
        used_libraries: Set[Path] = set()

        self._generate_ld_config_cache()

        for elf_file in elf_files:
            # Skip linting files listed in the ignore list for the main "library"
            # filter.
            if self._is_file_ignored(elf_file):
                continue

            arch_triplet = elf_utils.get_arch_triplet()
            content_dirs = self._snap_metadata.get_provider_content_directories()

            # if the elf file is a library, add it to the list of all libraries
            if elf_file.soname and self._is_library_path(elf_file.path):
                # resolve symlinks to libraries
                all_libraries.add(elf_file.path.resolve())

            dependencies = elf_file.load_dependencies(
                root_path=current_path.absolute(),
                base_path=installed_base_path,
                content_dirs=content_dirs,
                arch_triplet=arch_triplet,
                soname_cache=soname_cache,
            )

            # collect paths to local libraries used by the elf file
            for dependency in dependencies:
                if (
                    self._is_library_path(path=dependency)
                    and current_path.resolve() in dependency.resolve().parents
                ):
                    # resolve symlinks to libraries
                    used_libraries.add(dependency.resolve())

            # Check whether all dependencies are satisfied, *if* the missing-library
            # category is not filtered out for the elf file's path.
            if not self._is_file_ignored(elf_file, "missing-library"):
                search_paths = [current_path.absolute(), *content_dirs]
                if installed_base_path:
                    search_paths.append(installed_base_path)

                self._check_dependencies_satisfied(
                    elf_file,
                    search_paths=search_paths,
                    dependencies=sorted(dependencies),
                    issues=issues,
                )

        issues.extend(self._get_unused_library_issues(all_libraries, used_libraries))

        return issues

    def _generate_ld_config_cache(self) -> None:
        """Generate a cache of ldconfig output that maps library names to paths."""
        # Match lines like:
        # libcurl.so.4 (libc6,x86-64) => /lib/x86_64-linux-gnu/libcurl.so.4
        # Ignored any architecture in it, may be a problem in the future?
        ld_regex = re.compile(r"^\s*(\S+)\s+\(.*\)\s+=>\s+(\S+)$")

        try:
            output = subprocess.run(
                ["ldconfig", "-N", "-p"],
                check=True,
                stdout=subprocess.PIPE,
            )
        except subprocess.CalledProcessError:
            return

        for line in output.stdout.decode("UTF-8").splitlines():
            match = ld_regex.match(line)
            if match:
                self._ld_config_cache[match.group(1)] = Path(match.group(2))

    def _find_deb_package(self, library_name: str) -> Optional[str]:
        """Find the deb package that provides a library.

        :param library_name: The filename of the library to find.

        :returns: the corresponding deb package name, or None if the library
        is not provided by any system package.
        """
        if library_name in self._ld_config_cache:
            # Must be resolved to an absolute path for dpkg to find it
            library_absolute_path = self._ld_config_cache[library_name].resolve()
            try:
                output = subprocess.run(
                    ["dpkg", "-S", library_absolute_path.as_posix()],
                    check=True,
                    stdout=subprocess.PIPE,
                )
            except subprocess.CalledProcessError:
                # If the specified file doesn't belong to any package, the
                # call will trigger an exception.
                return None
            except FileNotFoundError:
                # In case that dpkg isn't available
                return None
            return output.stdout.decode("UTF-8").split(":", maxsplit=1)[0]
        return None

    def _check_dependencies_satisfied(
        self,
        elf_file: ElfFile,
        *,
        search_paths: List[Path],
        dependencies: List[Path],
        issues: List[LinterIssue],
    ) -> None:
        """Check if ELF executable dependencies are satisfied by snap files.

        :param elf_file: The ELF file whose dependencies are being verified.
        :param root_path: The absolute path to the payload directory.
        :param issues: The list of linter issues.
        """
        try:
            linker = elf_utils.get_dynamic_linker(root_path=Path("/"), snap_path=Path())
            linker_name = Path(linker).name
        except elf_errors.DynamicLinkerNotFound:
            linker_name = None

        emit.debug(f"dynamic linker name is: {linker_name!r}")

        for dependency in dependencies:
            # the dynamic linker is not a regular library
            if linker_name == dependency.name:
                continue

            for path in search_paths:
                if path in dependency.parents:
                    break
            else:
                deb_package = self._find_deb_package(dependency.name)
                message = f"missing dependency {dependency.name!r}."
                if deb_package:
                    message += f" (provided by '{deb_package}')"
                issue = LinterIssue(
                    name=self._name,
                    result=LinterResult.WARNING,
                    filename=str(elf_file.path),
                    text=message,
                    url="https://snapcraft.io/docs/linters-library",
                )
                issues.append(issue)

    def _get_unused_library_issues(
        self, all_libraries: Set[Path], used_libraries: Set[Path]
    ) -> List[LinterIssue]:
        """Get a list of unused library issues.

        :param all_libraries: a set of paths to all libraries
        :param used_libraries: a set of libraries used by elf files in the snap

        :returns: list of LinterIssues for unused libraries
        """
        issues: List[LinterIssue] = []
        unused_libraries = all_libraries - used_libraries

        # sort libraries so the results are ordered in a deterministic way
        for library_path in sorted(unused_libraries):
            try:
                # Resolving symlinks to a library will change the path from relative
                # to absolute. To make it relative again, remove the current directory
                # prefix from the path.
                resolved_library_path = library_path.resolve().relative_to(Path.cwd())
            except ValueError:
                # A ValueError is not expected because these libraries should be within
                # the current directory, but check anyways
                emit.debug(f"could not resolve path for library {library_path!r}")
                continue

            # skip linting files listed in the ignore list
            if self._is_file_ignored(resolved_library_path, "unused-library"):
                continue

            library = ElfFile(path=resolved_library_path)

            issue = LinterIssue(
                name=self._name,
                result=LinterResult.WARNING,
                filename=library.soname,
                text=f"unused library {str(library.path)!r}.",
                url="https://snapcraft.io/docs/linters-library",
            )
            issues.append(issue)

        return issues

    def _is_library_path(self, path: Path) -> bool:
        """Check if a file is in a library directory.

        An elf file is considered a library if it has an soname and it is within a
        library directory.
        A library directory is a directory whose pathname ends in `lib/` or
        `lib/<architecture-triplet>/` (i.e. lib/x86_64-linux-gnu/`)

        :param path: filepath to check

        :returns: True if the file is in a library directory. False if the path is not
        a file or if it is not in a library directory.
        """
        # follow symlinks
        path = path.resolve()

        if not path.is_file():
            return False

        # TODO: get library directories from LD_LIBRARY_PATH and `ld --verbose`
        # instead of matching against patterns
        if path.match("lib/*") or path.match("lib32/*") or path.match("lib64/*"):
            return True

        for arch_triplet in elf_utils.get_all_arch_triplets():
            if path.match(f"lib/{arch_triplet}/*"):
                return True

        return False
