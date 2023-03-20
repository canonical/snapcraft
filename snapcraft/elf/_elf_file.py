# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2023 Canonical Ltd.
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

"""Helpers to parse and handle ELF binary files."""

import contextlib
import os
import re
import subprocess
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

from craft_cli import emit
from elftools import elf
from elftools.construct import ConstructError
from pkg_resources import parse_version

from snapcraft import utils

from . import errors

_ElfArchitectureTuple = Tuple[str, str, str]
_SonameCacheDict = Dict[Tuple[_ElfArchitectureTuple, str], Path]

_DEBUG_INFO = ".debug_info"
_DYNAMIC = ".dynamic"
_GNU_VERSION_D = ".gnu.version_d"
_GNU_VERSION_R = ".gnu.version_r"
_INTERP = ".interp"


class _NeededLibrary:
    """Represents an ELF library version."""

    def __init__(self, *, name: str) -> None:
        self.name = name
        self.versions: Set[str] = set()

    def add_version(self, version: str) -> None:
        """Add a new library version."""
        self.versions.add(version)


class SonameCache:
    """A cache for sonames."""

    def __init__(self):
        self._soname_paths: _SonameCacheDict = {}

    def __getitem__(self, key):
        """Obtain cached item."""
        return self._soname_paths[key]

    def __setitem__(self, key: Tuple[_ElfArchitectureTuple, str], item: Path):
        """Add item to cache."""
        # Initial API error checks
        if not isinstance(key, tuple):
            raise EnvironmentError(
                "The key for SonameCache has to be a (arch, soname) tuple."
            )
        if not isinstance(key[0], tuple) or len(key[0]) != 3:
            raise EnvironmentError(
                "The first element of the key needs to of type _ElfArchitectureTuple."
            )
        if not isinstance(key[1], str):
            raise EnvironmentError(
                "The second element of the key needs to be "
                "of type str representing the soname."
            )
        self._soname_paths[key] = item

    def __contains__(self, key) -> bool:
        """Check if key is already in cache."""
        return key in self._soname_paths

    def reset_except_root(self, root: str) -> None:
        """Reset the cache values that aren't contained within root."""
        new_soname_paths: _SonameCacheDict = {}
        for key, value in self._soname_paths.items():
            if value is not None and str(value).startswith(root):
                new_soname_paths[key] = value

        self._soname_paths = new_soname_paths


class _Library:
    """Represents the soname and path to the library.

    :param soname: The library soname.
    :param soname_path: The full path to the version-named library.
    :param search_paths: Library search paths.
    :param base_path: The core base path to search for missing dependencies.
    :param arch_tuple: A tuple that identifies the architecture of the ELF file,
        containing the class, data byte order, and machine instruction set
        (e.g. ``(ELFCLASS64, ELFDATA2LSB, EM_X86_64)``).
    :param soname_cache: The soname cache manager.
    """

    def __init__(
        self,
        *,
        soname: str,
        soname_path: Path,
        search_paths: List[Path],
        base_path: Optional[Path],
        arch_tuple: _ElfArchitectureTuple,
        soname_cache: SonameCache,
    ) -> None:
        self.soname = soname
        self.soname_path = soname_path
        self.search_paths = search_paths
        self.base_path = base_path
        self.arch_tuple = arch_tuple
        self.soname_cache = soname_cache

        # Resolve path, if possible.
        self.path = self._crawl_for_path()

        if base_path is not None and base_path in self.path.parents:
            self.in_base_snap = True
        else:
            self.in_base_snap = False

        emit.debug(
            f"{soname} with original path {soname_path} found on {str(self.path)!r} "
            f"in base {self.in_base_snap!r}"
        )

    def _update_soname_cache(self, resolved_path: Path) -> None:
        self.soname_cache[self.arch_tuple, self.soname] = resolved_path

    def _is_valid_elf(self, resolved_path: Path) -> bool:
        if not resolved_path.exists() or not ElfFile.is_elf(resolved_path):
            return False

        try:
            elf_file = ElfFile(path=resolved_path)
        except errors.CorruptedElfFile as error:
            # Warn if the ELF file seems corrupted.
            emit.progress(str(error), permanent=True)
            return False

        return elf_file.arch_tuple == self.arch_tuple

    def _crawl_for_path(self) -> Path:
        # Speed things up and return what was already found once.
        if (self.arch_tuple, self.soname) in self.soname_cache:
            return self.soname_cache[self.arch_tuple, self.soname]

        emit.debug(f"Crawling to find soname {self.soname!r}")

        valid_search_paths = [p for p in self.search_paths if p.exists()]
        in_search_paths = any(p in self.soname_path.parents for p in valid_search_paths)

        # Expedite path crawling if we have a valid elf file that lives
        # inside the search paths.
        if in_search_paths and self._is_valid_elf(self.soname_path):
            self._update_soname_cache(self.soname_path)
            return self.soname_path

        for path in valid_search_paths:
            for root, _, files in os.walk(path):
                if self.soname not in files:
                    continue

                file_path = Path(root, self.soname.lstrip("/"))
                if self._is_valid_elf(file_path):
                    self._update_soname_cache(file_path)
                    return file_path

        # Required for libraries on the host and the fetching mechanism.
        self._update_soname_cache(self.soname_path)
        return self.soname_path


class ElfFile:
    """ElfFile represents and elf file on a path and its attributes."""

    def __init__(self, *, path: Path) -> None:
        """Initialize an ElfFile instance.

        :param str path: path to an elf_file within a snapcraft project.
        """
        self.path = path
        self.dependencies: Set[_Library] = set()

        self.arch_tuple: Optional[_ElfArchitectureTuple] = None
        self.interp = ""
        self.soname = ""
        self.versions: Set[str] = set()
        self.needed: Dict[str, _NeededLibrary] = {}
        self.execstack_set = False
        self.is_dynamic = True
        self.build_id = ""
        self.has_debug_info: bool = False

        self._required_glibc = ""

        # String of elf enum type, e.g. "ET_DYN", "ET_EXEC", etc.
        self.elf_type: str = "ET_NONE"

        try:
            emit.debug(f"Extracting ELF attributes: {str(path)!r}")
            self._extract_attributes()
        except (UnicodeDecodeError, AttributeError, ConstructError) as exception:
            emit.debug(f"Extracting ELF attributes exception: {str(exception)}")
            raise errors.CorruptedElfFile(path, exception)

    @classmethod
    def is_elf(cls, path: Path) -> bool:
        """Determine whether the given file is an ELF file.

        :param path: Path to the file to be verified.
        """
        if not path.is_file():
            # ELF binaries are regular files
            return False

        with path.open("rb") as bin_file:
            return bin_file.read(4) == b"\x7fELF"

    # pylint: disable=too-many-branches

    def _extract_attributes(self) -> None:  # noqa: C901
        with self.path.open("rb") as file:
            elf_file = elf.elffile.ELFFile(file)

            # A set of fields to identify the architecture of the ELF file:
            #  EI_CLASS: 32/64 bit (e.g. amd64 vs. x32)
            #  EI_DATA: byte order (e.g. ppc64 vs. ppc64le)
            #  e_machine: instruction set (e.g. x86-64 vs. arm64)
            #
            # For amd64 binaries, this will evaluate to:
            #   ('ELFCLASS64', 'ELFDATA2LSB', 'EM_X86_64')
            self.arch_tuple = (
                elf_file.header.e_ident.EI_CLASS,
                elf_file.header.e_ident.EI_DATA,
                elf_file.header.e_machine,
            )

            # Gather attributes from dynamic sections.
            for section in elf_file.iter_sections():
                if not isinstance(section, elf.dynamic.DynamicSection):
                    continue

                self.is_dynamic = True

                for tag in section.iter_tags():
                    if tag.entry.d_tag == "DT_NEEDED":
                        self.needed[tag.needed] = _NeededLibrary(name=tag.needed)
                    elif tag.entry.d_tag == "DT_SONAME":
                        self.soname = tag.soname

            for segment in elf_file.iter_segments():
                if segment["p_type"] == "PT_GNU_STACK":
                    # p_flags holds the bit mask for this segment.
                    # See `man 5 elf`.
                    mode = segment["p_flags"]
                    if mode & elf.constants.P_FLAGS.PF_X:
                        self.execstack_set = True
                elif isinstance(segment, elf.segments.InterpSegment):
                    self.interp = segment.get_interp_name()

            build_id_section = elf_file.get_section_by_name(".note.gnu.build-id")
            if (
                isinstance(build_id_section, elf.sections.NoteSection)
                and build_id_section.header["sh_type"] != "SHT_NOBITS"
            ):
                for note in build_id_section.iter_notes():
                    if note.n_name == "GNU" and note.n_type == "NT_GNU_BUILD_ID":
                        self.build_id = note.n_desc

            # If we are processing a detached debug info file, these
            # sections will be present but empty.
            verneed_section = elf_file.get_section_by_name(_GNU_VERSION_R)
            if isinstance(verneed_section, elf.gnuversions.GNUVerNeedSection):
                for library, versions in verneed_section.iter_versions():
                    library_name = library.name
                    # If the ELF file only references weak symbols
                    # from a library, it may be absent from DT_NEEDED
                    # but still have an entry in .gnu.version_r for
                    # symbol versions.
                    if library_name not in self.needed:
                        continue
                    lib = self.needed[library_name]
                    for version in versions:
                        lib.add_version(version.name)

            verdef_section = elf_file.get_section_by_name(_GNU_VERSION_D)
            if isinstance(verdef_section, elf.gnuversions.GNUVerDefSection):
                for _, auxiliaries in verdef_section.iter_versions():
                    for aux in auxiliaries:
                        self.versions.add(aux.name)

            debug_info_section = elf_file.get_section_by_name(_DEBUG_INFO)
            self.has_debug_info = (
                debug_info_section is not None
                and debug_info_section.header.sh_type != "SHT_NOBITS"
            )

            self.elf_type = elf_file.header["e_type"]

    # pylint: enable=too-many-branches

    def is_linker_compatible(self, *, linker_version: str) -> bool:
        """Determine if the linker will work given the required glibc version."""
        version_required = self.get_required_glibc()
        # TODO: pkg_resources is deprecated in setuptools>66 (CRAFT-1598)
        is_compatible = parse_version(version_required) <= parse_version(linker_version)
        emit.debug(
            f"Check if linker {linker_version!r} works with GLIBC_{version_required} "
            f"required by {str(self.path)!r}: {is_compatible}"
        )
        return is_compatible

    def get_required_glibc(self) -> str:
        """Return the required glibc version for this ELF file."""
        if self._required_glibc:
            return self._required_glibc

        version_required = ""
        for lib in self.needed.values():
            for version in lib.versions:
                if not version.startswith("GLIBC_"):
                    continue
                version = version[6:]
                # TODO: pkg_resources is deprecated in setuptools>66 (CRAFT-1598)
                if parse_version(version) > parse_version(version_required):
                    version_required = version

        self._required_glibc = version_required
        return version_required

    def load_dependencies(
        self,
        root_path: Path,
        base_path: Optional[Path],
        content_dirs: List[Path],
        arch_triplet: str,
        soname_cache: Optional[SonameCache] = None,
    ) -> Set[Path]:
        """Load the set of libraries that are needed to satisfy elf's runtime.

        This may include libraries contained within the project.
        The object's .dependencies attribute is set after loading.

        :param root_path: the root path to search for missing dependencies.
        :param base_path: the core base path to search for missing dependencies.
        :param content_dirs: list of paths sourced from content snaps.
        :param arch_triplet: architecture triplet of the platform.
        :param soname_cache: a cache of previously search dependencies.

        :returns: a set of paths to the library dependencies of elf.
        """
        if soname_cache is None:
            soname_cache = SonameCache()

        emit.debug(f"Getting dependencies for {str(self.path)!r}")

        search_paths = [root_path, *content_dirs]
        if base_path is not None:
            search_paths.append(base_path)

        ld_library_paths: List[str] = []
        for path in search_paths:
            ld_library_paths.extend(
                utils.get_common_ld_library_paths(path, arch_triplet)
            )

        libraries = _determine_libraries(
            path=self.path, ld_library_paths=ld_library_paths, arch_triplet=arch_triplet
        )
        for soname, soname_path in libraries.items():
            if self.arch_tuple is None:
                raise RuntimeError("failed to parse architecture")

            self.dependencies.add(
                _Library(
                    soname=soname,
                    soname_path=Path(soname_path),
                    search_paths=search_paths,
                    base_path=base_path,
                    arch_tuple=self.arch_tuple,
                    soname_cache=soname_cache,
                )
            )

        # Return the set of dependency paths, minus those found in the base.
        dependencies: Set[Path] = set()
        for library in self.dependencies:
            if not library.in_base_snap:
                dependencies.add(library.path)
        return dependencies


def _get_host_libc_path(arch_triplet) -> Path:
    return Path("/lib") / arch_triplet / "libc.so.6"


def _determine_libraries(
    *, path: Path, ld_library_paths: List[str], arch_triplet: str
) -> Dict[str, str]:
    # Try the usual method with ldd.
    with contextlib.suppress(subprocess.CalledProcessError):
        return _ldd(path, ld_library_paths)

    # Fall back to trying ldd with LD_PRELOAD explicitly loading libc.
    libc_path = _get_host_libc_path(arch_triplet)
    if libc_path.is_file():
        with contextlib.suppress(subprocess.CalledProcessError):
            return _ldd(path, ld_library_paths, ld_preload=str(libc_path))

    # Fall back to trying ld trace method which may fail with permission error
    # for non-executable shared objects, or OSError 8 Exec format error if
    # target is for different arch.
    with contextlib.suppress(PermissionError, OSError, subprocess.CalledProcessError):
        return _ld_trace(path, ld_library_paths)

    emit.progress(
        f"Unable to determine library dependencies for {str(path)!r}", permanent=True
    )

    return {}


def _ldd(
    path: Path, ld_library_paths: List[str], *, ld_preload: Optional[str] = None
) -> Dict[str, str]:
    """Use host ldd to determine library dependencies."""
    ldd = utils.get_host_tool("ldd")  # TODO: check if we can run from base
    env = {
        "LD_LIBRARY_PATH": ":".join(ld_library_paths),
    }

    if ld_preload:
        env["LD_PRELOAD"] = ld_preload

    return _parse_ldd_output(_check_output([ldd, str(path)], extra_env=env))


def _ld_trace(path: Path, ld_library_paths: List[str]) -> Dict[str, str]:
    """Use LD_TRACE_LOADED_OBJECTS to determine library dependencies."""
    env = {
        "LD_TRACE_LOADED_OBJECTS": "1",
        "LD_LIBRARY_PATH": ":".join(ld_library_paths),
    }

    return _parse_ldd_output(_check_output([str(path)], extra_env=env))


def _parse_ldd_output(output: str) -> Dict[str, str]:
    """Parse ldd output.

    Example ldd outputs:

    linux-vdso.so.1 =>  (0x00007ffdc13ec000)   <== ubuntu 16.04 ldd
    linux-vdso.so.1 (0x00007ffdc13ec000)       <== newer ldd
    /lib64/ld-linux-x86-64.so.2 (0x00007fb3c5298000)
    libm.so.6 => /lib/x86_64-linux-gnu/libm.so.6 (0x00007fb3bef03000)
    libmissing.so.2 => not found

    :returns: Dictionary of dependencies, mapping library name to path.
    """
    libraries: Dict[str, str] = {}
    ldd_lines = output.splitlines()

    for line in ldd_lines:
        # First match against libraries that are found.
        match = re.match(r"\t(.*) => (.*) \(0x", line)

        if not match:
            # Now find those not found, or not providing the address...
            match = re.match(r"\t(.*) => (.*)", line)

        # Ignore ld-linux, linux-vdso, etc. that don't match these regex.
        # As Ubuntu 16.04's ldd provides an empty string for the found
        # path (in group 2) on linux-vdso, check for this and ignore it.
        # See example output above for reference.
        if not match or match.group(2) == "":
            continue

        soname, soname_path = _ldd_resolve(match.group(1), match.group(2))
        libraries[soname] = soname_path

    return libraries


def _ldd_resolve(soname: str, soname_path: str) -> Tuple[str, str]:
    emit.debug(f"_ldd_resolve: {soname!r} {soname_path!r}")

    # If found, resolve the path components.  We can safely determine that
    # ldd found the match if it returns an absolute path.  For additional
    # safety, check that it exists.  See example ldd output in ldd() below.
    # If not found, ldd should use a string like "not found", but we do not
    # really care what that string is with this approach as it has to start
    # with "/" and point to a valid file.
    if soname_path.startswith("/") and os.path.exists(soname_path):
        abs_path = os.path.abspath(soname_path)
        return soname, abs_path

    # Not found, use the soname.
    return soname, soname


def _check_output(cmd: List[str], *, extra_env: Dict[str, str]) -> str:
    env = os.environ.copy()
    env.update(extra_env)

    debug_cmd = [f"{k}={v}" for k, v in extra_env.items()]
    debug_cmd += cmd

    emit.debug(f"executing: {' '.join(debug_cmd)}")
    output = subprocess.check_output(cmd, env=env).decode()

    return output
