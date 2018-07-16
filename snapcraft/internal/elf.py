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
import glob
import logging
import os
import re
import shutil
import subprocess
import tempfile
from typing import Dict, FrozenSet, List, Set, Sequence, Tuple, Union  # noqa

import elftools.elf.elffile
from pkg_resources import parse_version

from snapcraft import file_utils
from snapcraft.internal import common, errors, os_release, repo


logger = logging.getLogger(__name__)


class NeededLibrary:
    """Represents an ELF library version."""

    def __init__(self, *, name: str) -> None:
        self.name = name
        self.versions = set()  # type: Set[str]

    def add_version(self, version: str) -> None:
        self.versions.add(version)


ElfArchitectureTuple = Tuple[str, str, str]
ElfDataTuple = Tuple[
    ElfArchitectureTuple, str, str, Dict[str, NeededLibrary], bool
]  # noqa: E501
SonameCacheDict = Dict[Tuple[ElfArchitectureTuple, str], str]


# Old pyelftools uses byte strings for section names.  Some data is
# also returned as bytes, which is handled below.
if parse_version(elftools.__version__) >= parse_version("0.24"):
    _DYNAMIC = ".dynamic"  # type: Union[str, bytes]
    _GNU_VERSION_R = ".gnu.version_r"  # type: Union[str, bytes]
    _INTERP = ".interp"  # type: Union[str, bytes]
else:
    _DYNAMIC = b".dynamic"
    _GNU_VERSION_R = b".gnu.version_r"
    _INTERP = b".interp"


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
        path: str,
        root_path: str,
        core_base_path: str,
        arch: ElfArchitectureTuple,
        soname_cache: SonameCache
    ) -> None:
        self.soname = soname

        # We need to always look for the soname inside root first,
        # and after exhausting all options look in core_base_path.
        if path.startswith(root_path):
            self.path = path
        else:
            self.path = _crawl_for_path(
                soname=soname,
                root_path=root_path,
                core_base_path=core_base_path,
                arch=arch,
                soname_cache=soname_cache,
            )

        if not self.path and path.startswith(core_base_path):
            self.path = path

        # Required for libraries on the host and the fetching mechanism
        if not self.path:
            self.path = path

        system_libs = _get_system_libs()
        if soname in system_libs:
            self.system_lib = True
        else:
            self.system_lib = False

        # self.path has the correct resulting path.
        if self.path.startswith(core_base_path):
            self.in_base_snap = True
        else:
            self.in_base_snap = False


def _crawl_for_path(
    *,
    soname: str,
    root_path: str,
    core_base_path: str,
    arch: ElfArchitectureTuple,
    soname_cache: SonameCache
) -> str:
    # Speed things up and return what was already found once.
    if (arch, soname) in soname_cache:
        return soname_cache[arch, soname]

    logger.debug("Crawling to find soname {!r}".format(soname))
    for path in (root_path, core_base_path):
        if not os.path.exists(path):
            continue
        for root, directories, files in os.walk(path):
            for file_name in files:
                if file_name == soname:
                    file_path = os.path.join(root, file_name)
                    if ElfFile.is_elf(file_path):
                        # We found a match by name, anyway. Let's verify that
                        # the architecture is the one we want.
                        elf_file = ElfFile(path=file_path)
                        if elf_file.arch == arch:
                            soname_cache[arch, soname] = file_path
                            return file_path

    # If not found we cache it too
    soname_cache[arch, soname] = None
    return None


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
        elf_data = self._extract(path)
        self.arch = elf_data[0]
        self.interp = elf_data[1]
        self.soname = elf_data[2]
        self.needed = elf_data[3]
        self.execstack_set = elf_data[4]

    def _extract(self, path: str) -> ElfDataTuple:  # noqa: C901
        arch = None  # type: ElfArchitectureTuple
        interp = str()
        soname = str()
        libs = dict()
        execstack_set = False

        with open(path, "rb") as fp:
            elf = elftools.elf.elffile.ELFFile(fp)

            # A set of fields to identify the architecture of the ELF file:
            #  EI_CLASS: 32/64 bit (e.g. amd64 vs. x32)
            #  EI_DATA: byte orer (e.g. ppc64 vs. ppc64le)
            #  e_machine: instruction set (e.g. x86-64 vs. arm64)
            #
            # For amd64 binaries, this will evaluate to:
            #   ('ELFCLASS64', 'ELFDATA2LSB', 'EM_X86_64')
            arch = (
                elf.header.e_ident.EI_CLASS,
                elf.header.e_ident.EI_DATA,
                elf.header.e_machine,
            )

            # If we are processing a detached debug info file, these
            # sections will be present but empty.
            interp_section = elf.get_section_by_name(_INTERP)
            if (
                interp_section is not None
                and interp_section.header.sh_type != "SHT_NOBITS"
            ):
                interp = interp_section.data().rstrip(b"\x00").decode("ascii")

            dynamic_section = elf.get_section_by_name(_DYNAMIC)
            if (
                dynamic_section is not None
                and dynamic_section.header.sh_type != "SHT_NOBITS"
            ):
                for tag in dynamic_section.iter_tags("DT_NEEDED"):
                    needed = _ensure_str(tag.needed)
                    libs[needed] = NeededLibrary(name=needed)
                for tag in dynamic_section.iter_tags("DT_SONAME"):
                    soname = _ensure_str(tag.soname)

            verneed_section = elf.get_section_by_name(_GNU_VERSION_R)
            if (
                verneed_section is not None
                and verneed_section.header.sh_type != "SHT_NOBITS"
            ):
                for library, versions in verneed_section.iter_versions():
                    library_name = _ensure_str(library.name)
                    # If the ELF file only references weak symbols
                    # from a library, it may be absent from DT_NEEDED
                    # but still have an entry in .gnu.version_r for
                    # symbol versions.
                    if library_name not in libs:
                        continue
                    lib = libs[library_name]
                    for version in versions:
                        lib.add_version(_ensure_str(version.name))

            for segment in elf.iter_segments():
                if segment["p_type"] == "PT_GNU_STACK":
                    # p_flags holds the bit mask for this segment.
                    # See `man 5 elf`.
                    mode = segment["p_flags"]
                    if mode & elftools.elf.constants.P_FLAGS.PF_X:
                        execstack_set = True

        return arch, interp, soname, libs, execstack_set

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
        self, root_path: str, core_base_path: str, soname_cache: SonameCache = None
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
        ldd_out = []  # type: List[str]
        try:
            # ldd output sample:
            # /lib64/ld-linux-x86-64.so.2 (0x00007fb3c5298000)
            # libm.so.6 => /lib/x86_64-linux-gnu/libm.so.6 (0x00007fb3bef03000)
            ldd_out = common.run_output(["ldd", self.path]).split("\n")
        except subprocess.CalledProcessError:
            logger.warning(
                "Unable to determine library dependencies for {!r}".format(self.path)
            )
            return set()
        ldd_out_split = [l.split() for l in ldd_out]
        libs = set()
        for ldd_line in ldd_out_split:
            if len(ldd_line) > 2:
                libs.add(
                    Library(
                        soname=ldd_line[0],
                        path=ldd_line[2],
                        root_path=root_path,
                        core_base_path=core_base_path,
                        arch=self.arch,
                        soname_cache=soname_cache,
                    )
                )

        self.dependencies = libs

        # Return a set useful only for fetching libraries from the host
        library_paths = set()  # type: Set[str]
        for l in libs:
            if os.path.exists(l.path) and not l.in_base_snap and not l.system_lib:
                library_paths.add(l.path)
        return library_paths


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
            self._patchelf_cmd = file_utils.get_tool_path("patchelf")

        self._strip_cmd = file_utils.get_tool_path("strip")

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
        try:
            return self._do_run_patchelf(
                patchelf_args=patchelf_args, elf_file_path=elf_file_path
            )
        except errors.PatcherError as patch_error:
            # This is needed for patchelf to properly work with
            # go binaries (LP: #1736861).
            # We do this here instead of the go plugin for two reasons, the
            # first being that we do not want to blindly remove the section,
            # only doing it when necessary, and the second, this logic
            # should eventually be removed once patchelf catches up.
            try:
                logger.warning(
                    "Failed to update {!r}. Retrying after stripping "
                    "the .note.go.buildid from the elf file.".format(elf_file_path)
                )
                subprocess.check_call(
                    [
                        self._strip_cmd,
                        "--remove-section",
                        ".note.go.buildid",
                        elf_file_path,
                    ]
                )
            except subprocess.CalledProcessError:
                logger.warning(
                    "Could not properly strip .note.go.buildid "
                    "from {!r}.".format(elf_file_path)
                )
                raise patch_error
            return self._do_run_patchelf(
                patchelf_args=patchelf_args, elf_file_path=elf_file_path
            )

    def _do_run_patchelf(self, *, patchelf_args: List[str], elf_file_path: str) -> None:
        # Run patchelf on a copy of the primed file and replace it
        # after it is successful. This allows us to break the potential
        # hard link created when migrating the file across the steps of
        # the part.
        with tempfile.NamedTemporaryFile() as temp_file:
            shutil.copy2(elf_file_path, temp_file.name)

            cmd = [self._patchelf_cmd] + patchelf_args + [temp_file.name]
            try:
                subprocess.check_call(cmd)
            # There is no need to catch FileNotFoundError as patchelf should be
            # bundled with snapcraft which means its lack of existence is a
            # "packager" error.
            except subprocess.CalledProcessError as call_error:
                patchelf_version = (
                    subprocess.check_output([self._patchelf_cmd, "--version"])
                    .decode()
                    .strip()
                )
                # 0.10 is the version where patching certain binaries will
                # work (currently known affected packages are mostly built
                # with go).
                if parse_version(patchelf_version) < parse_version("0.10"):
                    raise errors.PatcherNewerPatchelfError(
                        elf_file=elf_file_path,
                        process_exception=call_error,
                        patchelf_version=patchelf_version,
                    )
                else:
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
        core_base_rpaths = ":".join(base_rpaths)

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


def _get_system_libs() -> FrozenSet[str]:
    global _libraries
    if _libraries:  # type: ignore
        return _libraries  # type: ignore

    lib_path = None

    release = os_release.OsRelease()
    with contextlib.suppress(errors.OsReleaseVersionIdError):
        lib_path = os.path.join(common.get_librariesdir(), release.version_id())

    if not lib_path or not os.path.exists(lib_path):
        logger.debug("Only excluding libc libraries from the release")
        libc6_libs = [
            os.path.basename(l) for l in repo.Repo.get_package_libraries("libc6")
        ]
        _libraries = frozenset(libc6_libs)
    else:
        with open(lib_path) as fn:
            _libraries = frozenset(fn.read().split())

    return _libraries


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
        # Finally, make sure this is actually an ELF file
        if ElfFile.is_elf(path):
            elf_file = ElfFile(path=path)
            # if we have dyn symbols we are dynamic
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
