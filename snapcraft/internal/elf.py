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
import functools
import glob
import logging
import os
import pathlib
import re
import shutil
import subprocess
import tempfile
from typing import Dict, FrozenSet, List, Optional, Sequence, Set, Tuple, Union

import elftools.common.exceptions
import elftools.elf.elffile
from elftools.construct import ConstructError
from pkg_resources import parse_version

from snapcraft import file_utils
from snapcraft.internal import common, errors, repo
from snapcraft.project._project_options import ProjectOptions

logger = logging.getLogger(__name__)


@functools.lru_cache()
def _get_host_libc_path() -> pathlib.Path:
    arch_triplet = ProjectOptions().arch_triplet
    return pathlib.Path("/lib") / arch_triplet / "libc.so.6"


def _ldd_resolve(soname: str, soname_path: str) -> Tuple[str, str]:
    logger.debug(f"_ldd_resolve: {soname!r} {soname_path!r}")

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

    logger.debug("executing: %s", " ".join(debug_cmd))
    output = subprocess.check_output(cmd, env=env).decode()

    return output


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


def _ld_trace(path: str, ld_library_paths: List[str]) -> Dict[str, str]:
    """Use LD_TRACE_LOADED_OBJECTS to determine library dependencies."""
    env = {
        "LD_TRACE_LOADED_OBJECTS": "1",
        "LD_LIBRARY_PATH": ":".join(ld_library_paths),
    }

    return _parse_ldd_output(_check_output([path], extra_env=env))


def _ldd(
    path: str, ld_library_paths: List[str], *, ld_preload: Optional[str] = None
) -> Dict[str, str]:
    """Use host ldd to determine library dependencies."""
    ldd_path = str(
        file_utils.get_host_tool_path(command_name="ldd", package_name="libc-bin")
    )
    env = {
        "LD_LIBRARY_PATH": ":".join(ld_library_paths),
    }

    if ld_preload:
        env["LD_PRELOAD"] = ld_preload

    return _parse_ldd_output(_check_output([ldd_path, path], extra_env=env))


def _determine_libraries(*, path: str, ld_library_paths: List[str]) -> Dict[str, str]:
    # Try the usual method with ldd.
    with contextlib.suppress(subprocess.CalledProcessError):
        return _ldd(path, ld_library_paths)

    # Fall back to trying ldd with LD_PRELOAD explicitly loading libc.
    libc_path = _get_host_libc_path()
    if libc_path.is_file():
        with contextlib.suppress(subprocess.CalledProcessError):
            return _ldd(path, ld_library_paths, ld_preload=str(libc_path))

    # Fall back to trying ld trace method which may fail with permission error
    # for non-executable shared objects, or OSError 8 Exec format error if
    # target is for different arch.
    with contextlib.suppress(PermissionError, OSError, subprocess.CalledProcessError):
        return _ld_trace(path, ld_library_paths)

    logger.warning("Unable to determine library dependencies for %r", path)
    return {}


class NeededLibrary:
    """Represents an ELF library version."""

    def __init__(self, *, name: str) -> None:
        self.name = name
        self.versions = set()  # type: Set[str]

    def add_version(self, version: str) -> None:
        self.versions.add(version)


ElfArchitectureTuple = Tuple[str, str, str]
SonameCacheDict = Dict[Tuple[ElfArchitectureTuple, str], str]


# Old pyelftools uses byte strings for section names.  Some data is
# also returned as bytes, which is handled below.
if parse_version(elftools.__version__) >= parse_version("0.24"):
    _DEBUG_INFO: Union[str, bytes] = ".debug_info"
    _DYNAMIC: Union[str, bytes] = ".dynamic"
    _GNU_VERSION_D: Union[str, bytes] = ".gnu.version_d"
    _GNU_VERSION_R: Union[str, bytes] = ".gnu.version_r"
    _INTERP: Union[str, bytes] = ".interp"
else:
    _DEBUG_INFO = b".debug_info"
    _GNU_VERSION_D = b".gnu.version_d"
    _GNU_VERSION_R = b".gnu.version_r"


class SonameCache:
    """A cache for sonames."""

    def __getitem__(self, key):
        return self._soname_paths[key]

    def __setitem__(self, key, item):
        # Initial API error checks
        if not isinstance(key, tuple):
            raise EnvironmentError(
                "The key for SonameCache has to be a (arch, soname) tuple."
            )
        if not isinstance(key[0], tuple) or len(key[0]) != 3:
            raise EnvironmentError(
                "The first element of the key needs to of type ElfArchitectureTuple."
            )
        if not isinstance(key[1], str):
            raise EnvironmentError(
                "The second element of the key needs to be "
                "of type str representing the soname."
            )
        self._soname_paths[key] = item

    def __contains__(self, key):
        return key in self._soname_paths

    def __init__(self):
        """Initialize a cache for sonames"""
        self._soname_paths = dict()  # type: SonameCacheDict

    def reset_except_root(self, root):
        """Reset the cache values that aren't contained within root."""
        new_soname_paths = dict()  # type: SonameCacheDict
        for key, value in self._soname_paths.items():
            if value is not None and value.startswith(root):
                new_soname_paths[key] = value

        self._soname_paths = new_soname_paths


class Library:
    """Represents the SONAME and path to the library."""

    def __init__(
        self,
        *,
        soname: str,
        soname_path: str,
        search_paths: List[str],
        core_base_path: Optional[str],
        arch: ElfArchitectureTuple,
        soname_cache: SonameCache,
    ) -> None:

        self.soname = soname
        self.soname_path = soname_path
        self.search_paths = search_paths
        self.core_base_path = core_base_path
        self.arch = arch
        self.soname_cache = soname_cache

        # Resolve path, if possible.
        self.path = self._crawl_for_path()

        if core_base_path is not None and self.path.startswith(core_base_path):
            self.in_base_snap = True
        else:
            self.in_base_snap = False

        logger.debug(
            "{soname} with original path {original_path} found on {path} in base: {in_base}".format(
                soname=soname,
                original_path=soname_path,
                path=self.path,
                in_base=self.in_base_snap,
            )
        )

    def _update_soname_cache(self, resolved_path: str) -> None:
        self.soname_cache[self.arch, self.soname] = resolved_path

    def _is_valid_elf(self, resolved_path: str) -> bool:
        if not os.path.exists(resolved_path) or not ElfFile.is_elf(resolved_path):
            return False

        try:
            elf_file = ElfFile(path=resolved_path)
        except errors.CorruptedElfFileError as error:
            # Log if the ELF file seems corrupted.
            logger.warning(error.get_brief())
            return False

        return elf_file.arch == self.arch

    def _crawl_for_path(self) -> str:
        # Speed things up and return what was already found once.
        if (self.arch, self.soname) in self.soname_cache:
            return self.soname_cache[self.arch, self.soname]

        logger.debug("Crawling to find soname {!r}".format(self.soname))

        valid_search_paths = [p for p in self.search_paths if os.path.exists(p)]
        in_search_paths = any(
            self.soname_path.startswith(p) for p in valid_search_paths
        )

        # Expedite path crawling if we have a valid elf file that lives
        # inside the search paths.
        if in_search_paths and self._is_valid_elf(self.soname_path):
            self._update_soname_cache(self.soname_path)
            return self.soname_path

        for path in valid_search_paths:
            for root, directories, files in os.walk(path):
                if self.soname not in files:
                    continue

                file_path = os.path.join(root, self.soname.strip("/"))
                if self._is_valid_elf(file_path):
                    self._update_soname_cache(file_path)
                    return file_path

        # Required for libraries on the host and the fetching mechanism.
        self._update_soname_cache(self.soname_path)
        return self.soname_path


# Old versions of pyelftools return bytes rather than strings for
# certain APIs.  So we pass those values through this function to get
# a consistent result.
def _ensure_str(s):
    if isinstance(s, bytes):
        return s.decode("ascii")
    assert isinstance(s, str)
    return s


class ElfFile:
    """ElfFile represents and elf file on a path and its attributes."""

    @classmethod
    def is_elf(cls, path: str) -> bool:
        if not os.path.isfile(path):
            # ELF binaries are regular files
            return False
        with open(path, "rb") as bin_file:
            return bin_file.read(4) == b"\x7fELF"

    def __init__(self, *, path: str) -> None:
        """Initialize an ElfFile instance.

        :param str path: path to an elf_file within a snapcraft project.
        """
        self.path = path
        self.dependencies = set()  # type: Set[Library]

        self.arch: Optional[ElfArchitectureTuple] = None
        self.interp: str = ""
        self.soname: str = ""
        self.versions: Set[str] = set()
        self.needed: Dict[str, NeededLibrary] = dict()
        self.execstack_set: bool = False
        self.is_dynamic: bool = True
        self.build_id: str = ""
        self.has_debug_info: bool = False

        # String of elf enum type, e.g. "ET_DYN", "ET_EXEC", etc.
        self.elf_type: str = "ET_NONE"

        try:
            logger.debug(f"Extracting ELF attributes: {path}")
            self._extract_attributes()
        except (UnicodeDecodeError, AttributeError, ConstructError) as exception:
            logger.debug(f"Extracting ELF attributes exception: {str(exception)}")
            raise errors.CorruptedElfFileError(path, exception)

    def _extract_attributes(self) -> None:  # noqa: C901
        with open(self.path, "rb") as fp:
            elf = elftools.elf.elffile.ELFFile(fp)

            # A set of fields to identify the architecture of the ELF file:
            #  EI_CLASS: 32/64 bit (e.g. amd64 vs. x32)
            #  EI_DATA: byte orer (e.g. ppc64 vs. ppc64le)
            #  e_machine: instruction set (e.g. x86-64 vs. arm64)
            #
            # For amd64 binaries, this will evaluate to:
            #   ('ELFCLASS64', 'ELFDATA2LSB', 'EM_X86_64')
            self.arch = (
                elf.header.e_ident.EI_CLASS,
                elf.header.e_ident.EI_DATA,
                elf.header.e_machine,
            )

            # Gather attributes from dynamic sections.
            for section in elf.iter_sections():
                if not isinstance(section, elftools.elf.dynamic.DynamicSection):
                    continue

                self.is_dynamic = True

                for tag in section.iter_tags():
                    if tag.entry.d_tag == "DT_NEEDED":
                        needed = _ensure_str(tag.needed)
                        self.needed[needed] = NeededLibrary(name=needed)
                    elif tag.entry.d_tag == "DT_SONAME":
                        self.soname = _ensure_str(tag.soname)

            for segment in elf.iter_segments():
                if segment["p_type"] == "PT_GNU_STACK":
                    # p_flags holds the bit mask for this segment.
                    # See `man 5 elf`.
                    mode = segment["p_flags"]
                    if mode & elftools.elf.constants.P_FLAGS.PF_X:
                        self.execstack_set = True
                elif isinstance(segment, elftools.elf.segments.InterpSegment):
                    self.interp = segment.get_interp_name()

            build_id_section = elf.get_section_by_name(".note.gnu.build-id")
            if (
                isinstance(build_id_section, elftools.elf.sections.NoteSection)
                and build_id_section.header["sh_type"] != "SHT_NOBITS"
            ):
                for note in build_id_section.iter_notes():
                    if note.n_name == "GNU" and note.n_type == "NT_GNU_BUILD_ID":
                        self.build_id = _ensure_str(note.n_desc)

            # If we are processing a detached debug info file, these
            # sections will be present but empty.
            verneed_section = elf.get_section_by_name(_GNU_VERSION_R)
            if isinstance(verneed_section, elftools.elf.gnuversions.GNUVerNeedSection):
                for library, versions in verneed_section.iter_versions():
                    library_name = _ensure_str(library.name)
                    # If the ELF file only references weak symbols
                    # from a library, it may be absent from DT_NEEDED
                    # but still have an entry in .gnu.version_r for
                    # symbol versions.
                    if library_name not in self.needed:
                        continue
                    lib = self.needed[library_name]
                    for version in versions:
                        lib.add_version(_ensure_str(version.name))

            verdef_section = elf.get_section_by_name(_GNU_VERSION_D)
            if isinstance(verdef_section, elftools.elf.gnuversions.GNUVerDefSection):
                for _, auxiliaries in verdef_section.iter_versions():
                    for aux in auxiliaries:
                        self.versions.add(_ensure_str(aux.name))

            debug_info_section = elf.get_section_by_name(_DEBUG_INFO)
            self.has_debug_info = (
                debug_info_section is not None
                and debug_info_section.header.sh_type != "SHT_NOBITS"
            )

            self.elf_type = elf.header["e_type"]

    def is_linker_compatible(self, *, linker_version: str) -> bool:
        """Determines if linker will work given the required glibc version."""
        version_required = self.get_required_glibc()
        r = parse_version(version_required) <= parse_version(linker_version)
        logger.debug(
            "Checking if linker {!r} will work with "
            "GLIBC_{} required by {!r}: {!r}".format(
                linker_version, version_required, self.path, r
            )
        )
        return r

    def get_required_glibc(self) -> str:
        """Returns the required glibc version for this ELF file."""
        with contextlib.suppress(AttributeError):
            return self._required_glibc  # type: ignore

        version_required = ""
        for lib in self.needed.values():
            for version in lib.versions:
                if not version.startswith("GLIBC_"):
                    continue
                version = version[6:]
                if parse_version(version) > parse_version(version_required):
                    version_required = version

        self._required_glibc = version_required
        return version_required

    def load_dependencies(
        self,
        root_path: str,
        core_base_path: Optional[str],
        content_dirs: Set[str],
        arch_triplet: str,
        soname_cache: SonameCache = None,
    ) -> Set[str]:
        """Load the set of libraries that are needed to satisfy elf's runtime.

        This may include libraries contained within the project.
        The object's .dependencies attribute is set after loading.

        :param str root_path: the root path to search for missing dependencies.
        :param str core_base_path: the core base path to search for missing
                                   dependencies.
        :param SonameCache soname_cache: a cache of previously search
                                         dependencies.
        :returns: a set of string with paths to the library dependencies of
                  elf.
        """
        if soname_cache is None:
            soname_cache = SonameCache()

        logger.debug("Getting dependencies for {!r}".format(self.path))

        search_paths = [root_path, *content_dirs]
        if core_base_path is not None:
            search_paths.append(core_base_path)

        ld_library_paths: List[str] = list()
        for path in search_paths:
            ld_library_paths.extend(common.get_library_paths(path, arch_triplet))

        libraries = _determine_libraries(
            path=self.path, ld_library_paths=ld_library_paths
        )
        for soname, soname_path in libraries.items():
            if self.arch is None:
                raise RuntimeError("failed to parse architecture")

            self.dependencies.add(
                Library(
                    soname=soname,
                    soname_path=soname_path,
                    search_paths=search_paths,
                    core_base_path=core_base_path,
                    arch=self.arch,
                    soname_cache=soname_cache,
                )
            )

        # Return the set of dependency paths, minus those found in the base.
        dependencies: Set[str] = set()
        for library in self.dependencies:
            if not library.in_base_snap:
                dependencies.add(library.path)
        return dependencies


class Patcher:
    """Patcher holds the necessary logic to patch elf files."""

    def __init__(
        self, *, dynamic_linker: str, root_path: str, preferred_patchelf_path=None
    ) -> None:
        """Create a Patcher instance.

        :param str dynamic_linker: the path to the dynamic linker to set the
                                   elf file to.
        :param str root_path: the base path for the snap to determine
                              if use of $ORIGIN is possible.
        :param str preferred_patchelf_path: patch the necessary elf_files with
                                        this patchelf.
        """
        self._dynamic_linker = dynamic_linker
        self._root_path = root_path

        if preferred_patchelf_path:
            self._patchelf_cmd = preferred_patchelf_path
        else:
            self._patchelf_cmd = file_utils.get_snap_tool_path("patchelf")

        self._strip_cmd = file_utils.get_snap_tool_path("strip")

    def patch(self, *, elf_file: ElfFile) -> None:
        """Patch elf_file with the Patcher instance configuration.

        If the ELF is executable, patch it to use the configured linker.
        If the ELF has dependencies (DT_NEEDED), set an rpath to them.

        :param ElfFile elf: a data object representing an elf file and its
                            relevant attributes.
        :raises snapcraft.internal.errors.PatcherError:
            raised when the elf_file cannot be patched.
        """
        patchelf_args = []
        if elf_file.interp:
            patchelf_args.extend(["--set-interpreter", self._dynamic_linker])
        if elf_file.dependencies:
            rpath = self._get_rpath(elf_file)
            # Due to https://github.com/NixOS/patchelf/issues/94 we need
            # to first clear the current rpath
            self._run_patchelf(
                patchelf_args=["--remove-rpath"], elf_file_path=elf_file.path
            )
            # Parameters:
            # --force-rpath: use RPATH instead of RUNPATH.
            # --shrink-rpath: will remove unneeded entries, with the
            #                 side effect of preferring host libraries
            #                 so we simply do not use it.
            # --set-rpath: set the RPATH to the colon separated argument.
            patchelf_args.extend(["--force-rpath", "--set-rpath", rpath])

        # no patchelf_args means there is nothing to do.
        if not patchelf_args:
            return

        self._run_patchelf(patchelf_args=patchelf_args, elf_file_path=elf_file.path)

    def _run_patchelf(self, *, patchelf_args: List[str], elf_file_path: str) -> None:
        # Run patchelf on a copy of the primed file and replace it
        # after it is successful. This allows us to break the potential
        # hard link created when migrating the file across the steps of
        # the part.
        with tempfile.NamedTemporaryFile() as temp_file:
            shutil.copy2(elf_file_path, temp_file.name)

            cmd = [self._patchelf_cmd] + patchelf_args + [temp_file.name]
            try:
                logger.debug("executing: %s", " ".join(cmd))
                subprocess.check_call(cmd)
            # There is no need to catch FileNotFoundError as patchelf should be
            # bundled with snapcraft which means its lack of existence is a
            # "packager" error.
            except subprocess.CalledProcessError as call_error:
                raise errors.PatcherGenericError(
                    elf_file=elf_file_path, process_exception=call_error
                )

            # We unlink to break the potential hard link
            os.unlink(elf_file_path)
            shutil.copy2(temp_file.name, elf_file_path)

    def _get_existing_rpath(self, elf_file_path):
        output = subprocess.check_output(
            [self._patchelf_cmd, "--print-rpath", elf_file_path]
        )
        return output.decode().strip().split(":")

    def _get_rpath(self, elf_file) -> str:
        origin_rpaths = list()  # type: List[str]
        base_rpaths = set()  # type: Set[str]
        existing_rpaths = self._get_existing_rpath(elf_file.path)

        for dependency in elf_file.dependencies:
            if dependency.path:
                if dependency.in_base_snap:
                    base_rpaths.add(os.path.dirname(dependency.path))
                elif dependency.path.startswith(self._root_path):
                    rel_library_path = os.path.relpath(dependency.path, elf_file.path)
                    rel_library_path_dir = os.path.dirname(rel_library_path)
                    # return the dirname, with the first .. replace
                    # with $ORIGIN
                    origin_rpath = rel_library_path_dir.replace("..", "$ORIGIN", 1)
                    if origin_rpath not in origin_rpaths:
                        origin_rpaths.append(origin_rpath)

        if existing_rpaths:
            # Only keep those that mention origin and are not already in our
            # bundle.
            existing_rpaths = [
                r for r in existing_rpaths if "$ORIGIN" in r and r not in origin_rpaths
            ]
            origin_rpaths = existing_rpaths + origin_rpaths

        origin_paths = ":".join((r for r in origin_rpaths if r))
        core_base_rpaths = ":".join(sorted(base_rpaths))

        if origin_paths and core_base_rpaths:
            return "{}:{}".format(origin_paths, core_base_rpaths)
        elif origin_paths and not core_base_rpaths:
            return origin_paths
        else:
            return core_base_rpaths


def determine_ld_library_path(root: str) -> List[str]:
    """Determine additional library paths needed for the linker loader.

    This is a workaround until full library searching is implemented which
    works by searching for ld.so.conf in specific hard coded locations
    within root.

    :param root str: the root directory to search for specific ld.so.conf
                     entries.
    :returns: a list of strings of library paths where relevant libraries
              can be found within root.
    """
    # If more ld.so.conf files need to be supported, add them here.
    ld_config_globs = {"{}/usr/lib/*/mesa*/ld.so.conf".format(root)}

    ld_library_paths = []
    for this_glob in ld_config_globs:
        for ld_conf_file in glob.glob(this_glob):
            ld_library_paths.extend(_extract_ld_library_paths(ld_conf_file))

    return [root + path for path in ld_library_paths]


def _extract_ld_library_paths(ld_conf_file: str) -> List[str]:
    # From the ldconfig manpage, paths can be colon-, space-, tab-, newline-,
    # or comma-separated.
    path_delimiters = re.compile(r"[:\s,]")
    comments = re.compile(r"#.*$")

    paths = []
    with open(ld_conf_file, "r") as f:
        for line in f:
            # Remove comments from line
            line = comments.sub("", line).strip()

            if line:
                paths.extend(path_delimiters.split(line))

    return paths


_libraries = None


def get_elf_files(root: str, file_list: Sequence[str]) -> FrozenSet[ElfFile]:
    """Return a frozenset of elf files from file_list prepended with root.

    :param str root: the root directory from where the file_list is generated.
    :param file_list: a list of file in root.
    :returns: a frozentset of ElfFile objects.
    """
    elf_files = set()  # type: Set[ElfFile]

    for part_file in file_list:
        # Filter out object (*.o) files-- we only care about binaries.
        if part_file.endswith(".o"):
            continue

        # No need to crawl links-- the original should be here, too.
        path = os.path.join(root, part_file)  # type: str
        if os.path.islink(path):
            logger.debug("Skipped link {!r} while finding dependencies".format(path))
            continue

        # Ignore if file does not have ELF header.
        if not ElfFile.is_elf(path):
            continue

        try:
            elf_file = ElfFile(path=path)
        except elftools.common.exceptions.ELFError:
            # Ignore invalid ELF files.
            continue
        except errors.CorruptedElfFileError as exception:
            # Log if the ELF file seems corrupted
            logger.warning(exception.get_brief())
            continue

        # If ELF has dynamic symbols, add it.
        if elf_file.needed:
            elf_files.add(elf_file)

    return frozenset(elf_files)


def _get_dynamic_linker(library_list: List[str]) -> str:
    """Return the dynamic linker from library_list."""
    regex = re.compile(r"(?P<dynamic_linker>ld-[\d.]+.so)$")

    for library in library_list:
        m = regex.search(os.path.basename(library))
        if m:
            return library

    raise RuntimeError(
        "The format for the linker should be of the form "
        "<root>/ld-<X>.<Y>.so. There are no matches for the "
        "current libc6 package"
    )


def find_linker(*, root_path: str, snap_base_path: str) -> str:
    """Find and return the dynamic linker that would be seen at runtime.

    :param str root_path: the root path of a snap tree.
    :param str snap_base_path: absolute path to the snap once installed to
                               setup proper rpaths.
    :returns: the path to the dynamic linker to use
    """
    # We assume the current system will satisfy the GLIBC requirement,
    # get the current libc6 libraries (which includes the linker)
    libc6_libraries_list = repo.Repo.get_package_libraries("libc6")

    # For security reasons, we do not want to automatically pull in
    # libraries but expect them to be consciously brought in by stage-packages
    # instead.
    libc6_libraries_paths = [
        os.path.join(root_path, l[1:]) for l in libc6_libraries_list
    ]

    dynamic_linker = _get_dynamic_linker(libc6_libraries_paths)

    # Get the path to the "would be" dynamic linker when this snap is
    # installed. Strip the root_path from the retrieved dynamic_linker
    # variables + the leading `/` so that os.path.join can perform the
    # proper join with snap_base_path.
    dynamic_linker_path = os.path.join(
        snap_base_path, dynamic_linker[len(root_path) + 1 :]
    )

    return dynamic_linker_path
