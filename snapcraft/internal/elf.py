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
from functools import wraps
from typing import FrozenSet, List, Set, Sequence, Tuple

from pkg_resources import parse_version

from snapcraft import file_utils
from snapcraft.internal import (
    common,
    errors,
    os_release,
    repo,
)


logger = logging.getLogger(__name__)


class Symbol:
    """Represents a symbol within an ELF file."""

    def __init__(self, *, name: str, version: str, section: str) -> None:
        self.name = name
        self.version = version
        self.section = section

    def is_undefined(self) -> bool:
        return self.section == 'UND'


class Library:
    """Represents the SONAME and path to the library."""

    def __init__(self, *, soname: str, path: str, root_path: str,
                 core_base_path: str) -> None:
        self.soname = soname
        if path.startswith(root_path) or path.startswith(core_base_path):
            self.path = path
        else:
            self.path = _crawl_for_path(soname=soname,
                                        root_path=root_path,
                                        core_base_path=core_base_path)
        # Required for libraries on the host and the fetching mechanism
        if not self.path:
            self.path = path

        system_libs = _get_system_libs()
        if soname in system_libs:
            self.system_lib = True
        else:
            self.system_lib = False

        if path.startswith(core_base_path):
            self.in_base_snap = True
        else:
            self.in_base_snap = False


def _crawl_for_path(*, soname: str, root_path: str,
                    core_base_path: str) -> str:
    for path in (root_path, core_base_path):
        if not os.path.exists(path):
            continue
        for root, directories, files in os.walk(path):
            for file_name in files:
                if file_name == soname:
                    file_path = os.path.join(root, file_name)
                    if os.path.exists(file_path) and ElfFile.is_elf(file_path):
                        return file_path
    return None


class ElfFile:
    """ElfFile represents and elf file on a path and its attributes."""

    @classmethod
    def is_elf(cls, path: str) -> bool:
        with open(path, 'rb') as bin_file:
            return bin_file.read(4) == b'\x7fELF'

    def __init__(self, *, path: str) -> None:
        """Initialize an ElfFile instance.

        :param str path: path to an elf_file within a snapcraft project.
        """
        self.path = path
        self.dependencies = set()  # type: Set[Library]
        self.interp, self.symbols = self._extract_readelf(path)

    def _extract_readelf(self, path) -> Tuple[str, List[Symbol]]:
        interp = str()
        symbols = list()  # type: List[Symbol]
        readelf_path = file_utils.get_tool_path('readelf')
        output = subprocess.check_output([
            readelf_path, '--wide', '--program-headers', '--dyn-syms', path])
        readelf_lines = output.decode(errors='surrogateescape').split('\n')
        # Regex inspired by the regexes in lintian to match entries similar to
        # the following sample output
        # Program Headers:
        #   Type           Offset   VirtAddr           PhysAddr           FileSiz  MemSiz   Flg Align     # noqa
        #   PHDR           0x41d000 0x000000000041d000 0x000000000041d000 0x000268 0x000268 R E 0x8       # noqa
        #   LOAD           0x000000 0x0000000000000000 0x0000000000000000 0x069904 0x069904 R E 0x200000  # noqa
        #   GNU_STACK      0x000000 0x0000000000000000 0x0000000000000000 0x000000 0x000000 RW  0x10      # noqa
        #   INTERP         0x000270 0x0000000000000270 0x0000000000000270 0x00001c 0x00001c R   0x1       # noqa
        #       [Requesting program interpreter: /lib64/ld-linux-x86-64.so.2]
        #   NOTE           0x00028c 0x000000000000028c 0x000000000000028c 0x000044 0x000044 R   0x4       # noqa
        #   GNU_EH_FRAME   0x05cb48 0x000000000005cb48 0x000000000005cb48 0x00189c 0x00189c R   0x4       # noqa
        #   LOAD           0x06a780 0x000000000026a780 0x000000000026a780 0x003a91 0x004c78 RW  0x200000  # noqa
        #   TLS            0x06a780 0x000000000026a780 0x000000000026a780 0x000128 0x000128 R   0x20      # noqa
        #   GNU_RELRO      0x06a780 0x000000000026a780 0x000000000026a780 0x003880 0x003880 R   0x1       # noqa
        #   LOAD           0x41d000 0x000000000041d000 0x000000000041d000 0x000b18 0x000b18 RW  0x1000    # noqa
        #   DYNAMIC        0x41d268 0x000000000041d268 0x000000000041d268 0x000220 0x000220 RW  0x8       # noqa
        #
        #  Section to Segment mapping:
        #   Segment Sections...
        #    00
        #    01     .interp .note.ABI-tag .note.gnu.build-id .gnu.hash .dynsym .gnu.version .gnu.version_r .rela.dyn .init .plt .plt.got .text .fini .rodata .eh_frame_hdr .eh_frame .gcc_except_table  # noqa
        #    02
        #    03     .interp
        #    04     .note.ABI-tag .note.gnu.build-id
        #    05     .eh_frame_hdr
        #    06     .tdata .init_array .fini_array .jcr .data.rel.ro .got .data .bss          # noqa
        #    07     .tdata
        #    08     .tdata .init_array .fini_array .jcr .data.rel.ro .got
        #    09     .dynamic .dynstr
        #    10     .dynamic
        # Symbol table '.dynsym' contains 2281 entries:
        #   Num:    Value          Size Type    Bind   Vis      Ndx Name
        #     0: 0000000000000000     0 NOTYPE  LOCAL  DEFAULT  UND
        #     1: 0000000000000000     0 FUNC    GLOBAL DEFAULT  UND endgrent@GLIBC_2.2.5 (2)  # noqa
        interp_match = (r'^\s+\[Requesting program interpreter:\s'
                        r'(?P<interp>[\S-]+)\]\Z')
        interp_regex = re.compile(interp_match)
        symbol_match = (r'^\s+\d+:\s+[0-9a-f]+\s+\d+\s+\S+\s+\S+\s+\S+\s+'
                        r'(?P<section>\S+)\s+(?P<symbol>\S+).*\Z')
        symbol_regex = re.compile(symbol_match)
        in_symbols_section = False
        for line in readelf_lines:
            if line.startswith("Symbol table '.dynsym'"):
                in_symbols_section = True
            if in_symbols_section:
                m = symbol_regex.search(line)
                if m:
                    section = m.group('section')
                    symbol = m.group('symbol')
                    try:
                        name, version = symbol.split('@')
                    except ValueError:
                        name = symbol
                        version = ''
                    symbols.append(Symbol(name=name, version=version,
                                          section=section))
            else:
                m = interp_regex.match(line)
                if m:
                    interp = m.group('interp')

        return interp, symbols

    def is_linker_compatible(self, *, linker: str) -> bool:
        """Determines if linker will work given the required glibc version.

        The linker passed needs to be of the format <root>/ld-<X>.<Y>.so.
        """
        version_required = self.get_required_glibc()
        m = re.search(r'ld-(?P<linker_version>[\d.]+).so$',
                      os.path.basename(linker))
        if not m:
            # This is a programmatic error, we don't want to be friendly
            # about this.
            raise EnvironmentError('The format for the linker should be of the'
                                   'form <root>/ld-<X>.<Y>.so. {!r} does not '
                                   'match that format.'.format(linker))
        linker_version = m.group('linker_version')
        r = parse_version(version_required) <= parse_version(linker_version)
        logger.debug('Checking if linker {!r} will work with '
                     'GLIBC_{}: {!r}'.format(linker, version_required, r))
        return r

    def get_required_glibc(self) -> str:
        """Returns the required glibc version for this ELF file."""
        with contextlib.suppress(AttributeError):
            return self._required_glibc  # type: ignore

        version_required = ''
        for symbol in self.symbols:
            if not symbol.is_undefined():
                continue
            if not symbol.version.startswith('GLIBC_'):
                continue
            version = symbol.version[6:]
            if parse_version(version) > parse_version(version_required):
                version_required = version

        self._required_glibc = version_required
        return version_required

    def load_dependencies(self, root_path: str,
                          core_base_path: str) -> Set[str]:
        """Load the set of libraries that are needed to satisfy elf's runtime.

        This may include libraries contained within the project.
        The object's .dependencies attribute is set after loading.

        :param str root_path: the base path to search for missing dependencies.
        :returns: a set of string with paths to the library dependencies of
                  elf.
        """
        logger.debug('Getting dependencies for {!r}'.format(self.path))
        ldd_out = []  # type: List[str]
        try:
            # ldd output sample:
            # /lib64/ld-linux-x86-64.so.2 (0x00007fb3c5298000)
            # libm.so.6 => /lib/x86_64-linux-gnu/libm.so.6 (0x00007fb3bef03000)
            ldd_out = common.run_output(['ldd', self.path]).split('\n')
        except subprocess.CalledProcessError:
            logger.warning(
                'Unable to determine library dependencies for '
                '{!r}'.format(self.path))
            return set()
        ldd_out_split = [l.split() for l in ldd_out]
        libs = set()
        for ldd_line in ldd_out_split:
            if len(ldd_line) > 2:
                libs.add(Library(soname=ldd_line[0],
                                 path=ldd_line[2],
                                 root_path=root_path,
                                 core_base_path=core_base_path))

        self.dependencies = libs

        # Return a set useful only for fetching libraries from the host
        library_paths = set()  # type: Set[str]
        for l in libs:
            if (os.path.exists(l.path) and
                    not l.in_base_snap and
                    not l.system_lib):
                library_paths.add(l.path)
        return library_paths


def _retry_patch(f):
    @wraps(f)
    def wrapper(*args, **kwargs):
        try:
            return f(*args, **kwargs)
        except errors.PatcherError as patch_error:
            # This is needed for patchelf to properly work with
            # go binaries (LP: #1736861).
            # We do this here instead of the go plugin for two reasons, the
            # first being that we do not want to blindly remove the section,
            # only doing it when necessary, and the second, this logic
            # should eventually be removed once patchelf catches up.
            try:
                elf_file_path = kwargs['elf_file_path']
                logger.info('Failed to update {!r}. Retrying after stripping '
                            'the .note.go.buildid from the elf file.'.format(
                                elf_file_path))
                subprocess.check_call([
                    'strip', '--remove-section', '.note.go.buildid',
                    elf_file_path])
            except subprocess.CalledProcessError:
                logger.warning('Could not properly strip .note.go.buildid '
                               'from {!r}.'.format(elf_file_path))
                raise patch_error
            return f(*args, **kwargs)
    return wrapper


class Patcher:
    """Patcher holds the necessary logic to patch elf files."""

    def __init__(self, *, dynamic_linker: str, root_path: str,
                 preferred_patchelf_path=None) -> None:
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

        # We will first fallback to the preferred_patchelf_path,
        # if that is not found we will look for the snap and finally,
        # if we are running from the snap we want to use the patchelf
        # bundled there as it would have the capability of working
        # anywhere given the fixed ld it would have.
        # If not found, resort to whatever is on the system brought
        # in by packaging dependencies.
        # The docker conditional will work if the docker image has the
        # snaps unpacked in the corresponding locations.
        if preferred_patchelf_path:
            self._patchelf_cmd = preferred_patchelf_path
        # We use the full path here as the path may not be set on
        # build systems where the path is recently created and added
        # to the environment
        elif os.path.exists('/snap/bin/patchelf'):
            self._patchelf_cmd = '/snap/bin/patchelf'
        elif common.is_snap():
            snap_dir = os.getenv('SNAP')
            self._patchelf_cmd = os.path.join(snap_dir, 'bin', 'patchelf')
        elif (common.is_docker_instance() and
              os.path.exists('/snap/snapcraft/current/bin/patchelf')):
            self._patchelf_cmd = '/snap/snapcraft/current/bin/patchelf'
        else:
            self._patchelf_cmd = 'patchelf'

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
            patchelf_args.extend(['--set-interpreter',  self._dynamic_linker])
        if elf_file.dependencies:
            # Parameters:
            # --force-rpath: use RPATH instead of RUNPATH.
            # --shrink-rpath: will remove unneeded entries, with the
            #                 side effect of preferring host libraries
            #                 so we simply do not use it.
            # --set-rpath: set the RPATH to the colon separated argument.
            rpath = self._get_rpath(elf_file)
            patchelf_args.extend(['--force-rpath', '--set-rpath', rpath])

        # no patchelf_args means there is nothing to do.
        if not patchelf_args:
            return

        self._run_patchelf(patchelf_args=patchelf_args,
                           elf_file_path=elf_file.path)

    @_retry_patch
    def _run_patchelf(self, *, patchelf_args: List[str],
                      elf_file_path: str) -> None:

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
                patchelf_version = subprocess.check_output(
                    [self._patchelf_cmd, '--version']).decode().strip()
                # 0.10 is the version where patching certain binaries will
                # work (currently known affected packages are mostly built
                # with go).
                if parse_version(patchelf_version) < parse_version('0.10'):
                    raise errors.PatcherNewerPatchelfError(
                        elf_file=elf_file_path,
                        process_exception=call_error,
                        patchelf_version=patchelf_version)
                else:
                    raise errors.PatcherGenericError(
                        elf_file=elf_file_path,
                        process_exception=call_error)

            # We unlink to break the potential hard link
            os.unlink(elf_file_path)
            shutil.copy2(temp_file.name, elf_file_path)

    def _get_existing_rpath(self, elf_file_path):
        output = subprocess.check_output([self._patchelf_cmd, '--print-rpath',
                                          elf_file_path])
        return output.decode().strip().split(':')

    def _get_rpath(self, elf_file) -> str:
        origin_rpaths = list()  # type: List[str]
        base_rpaths = set()  # type: Set[str]
        existing_rpaths = self._get_existing_rpath(elf_file.path)

        for dependency in elf_file.dependencies:
            if dependency.path:
                if dependency.in_base_snap:
                    base_rpaths.add(os.path.dirname(dependency.path))
                elif dependency.path.startswith(self._root_path):
                    rel_library_path = os.path.relpath(dependency.path,
                                                       elf_file.path)
                    rel_library_path_dir = os.path.dirname(rel_library_path)
                    # return the dirname, with the first .. replace
                    # with $ORIGIN
                    origin_rpaths.append(rel_library_path_dir.replace(
                        '..', '$ORIGIN', 1))

        if existing_rpaths:
            # Only keep those that mention origin and are not already in our
            # bundle.
            existing_rpaths = [r for r in existing_rpaths
                               if '$ORIGIN' in r and r not in origin_rpaths]
            origin_rpaths = existing_rpaths + origin_rpaths

        origin_paths = ':'.join((r for r in origin_rpaths if r))
        core_base_rpaths = ':'.join(base_rpaths)

        if origin_paths and core_base_rpaths:
            return '{}:{}'.format(origin_paths, core_base_rpaths)
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
    ld_config_globs = {
        '{}/usr/lib/*/mesa*/ld.so.conf'.format(root)
    }

    ld_library_paths = []
    for this_glob in ld_config_globs:
        for ld_conf_file in glob.glob(this_glob):
            ld_library_paths.extend(_extract_ld_library_paths(ld_conf_file))

    return [root + path for path in ld_library_paths]


def _extract_ld_library_paths(ld_conf_file: str) -> List[str]:
    # From the ldconfig manpage, paths can be colon-, space-, tab-, newline-,
    # or comma-separated.
    path_delimiters = re.compile(r'[:\s,]')
    comments = re.compile(r'#.*$')

    paths = []
    with open(ld_conf_file, 'r') as f:
        for line in f:
            # Remove comments from line
            line = comments.sub('', line).strip()

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
        lib_path = os.path.join(
            common.get_librariesdir(), release.version_id())

    if not lib_path or not os.path.exists(lib_path):
        logger.debug('Only excluding libc libraries from the release')
        libc6_libs = [os.path.basename(l)
                      for l in repo.Repo.get_package_libraries('libc6')]
        return frozenset(libc6_libs)

    with open(lib_path) as fn:
        _libraries = frozenset(fn.read().split())

    return _libraries


def _is_dynamically_linked_elf(file_m: str) -> bool:
    return file_m.startswith('ELF') and 'dynamically linked' in file_m


def get_elf_files(root: str,
                  file_list: Sequence[str]) -> FrozenSet[ElfFile]:
    """Return a frozenset of elf files from file_list prepended with root.

    :param str root: the root directory from where the file_list is generated.
    :param file_list: a list of file in root.
    :returns: a frozentset of ElfFile objects.
    """
    elf_files = set()  # type: Set[ElfFile]

    for part_file in file_list:
        # Filter out object (*.o) files-- we only care about binaries.
        if part_file.endswith('.o'):
            continue

        # No need to crawl links-- the original should be here, too.
        path = os.path.join(root, part_file)  # type: str
        if os.path.islink(path):
            logger.debug('Skipped link {!r} while finding dependencies'.format(
                path))
            continue
        # Finally, make sure this is actually an ELF file
        if ElfFile.is_elf(path):
            elf_file = ElfFile(path=path)
            # if we have dyn symbols we are dynamic
            if elf_file.symbols:
                elf_files.add(ElfFile(path=path))

    return frozenset(elf_files)
