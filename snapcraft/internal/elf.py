# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016-2017 Canonical Ltd
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
import re
import glob
import logging
import os
import subprocess
import sys
from typing import FrozenSet, List, Set, Sequence

import magic

from snapcraft.internal import (
    common,
    errors,
    os_release,
    repo,
)


logger = logging.getLogger(__name__)


class ElfFile:
    """ElfFile represents and elf file on a path and its attributes."""

    def __init__(self, *, path: str, is_executable: bool=True) -> None:
        """Initialize an ElfFile instance.

        :param str path: path to an elf_file within a snapcraft project.
        :param bool is_executable: True if the elf_file is an executable,
                                   meaning that it has in .interp entry
                                   in its headers.
        """
        self.path = path
        self.is_executable = is_executable
        self.dependencies = set()  # type: Set[str]

    def load_dependencies(self) -> Set[str]:
        """Load the set of libraries that are needed to satisfy elf's runtime.

        This may include libraries contained within the project.
        The object's .dependencies attribute is set after loading.

        :returns: a set of string with paths to the library dependencies of
                  elf.
        """
        logger.debug('Getting dependencies for {!r}'.format(self.path))
        ldd_out = []  # type: List[str]
        try:
            ldd_out = common.run_output(['ldd', self.path]).split('\n')
        except subprocess.CalledProcessError:
            logger.warning(
                'Unable to determine library dependencies for '
                '{!r}'.format(self.path))
            return set()
        ldd_out_split = [l.split() for l in ldd_out]
        ldd_out_list = [l[2] for l in ldd_out_split
                        if len(l) > 2 and os.path.exists(l[2])]

        # Now lets filter out what would be on the system
        system_libs = _get_system_libs()
        libs = {l for l in ldd_out_list
                if not os.path.basename(l) in system_libs}
        # Classic confinement won't do what you want with library crawling
        # and has a nasty side effect described in LP: #1670100
        libs = {l for l in libs if not l.startswith('/snap/')}

        self.dependencies = libs
        return libs


class Patcher:
    """Patcher holds the necessary logic to patch elf files."""

    def __init__(self, *, dynamic_linker: str,
                 library_path_func=lambda x: x,
                 base_rpaths: List[str]= None) -> None:
        """Create a Patcher instance.

        :param str dynamic_linker: the path to the dynamic linker to set the
                                   elf file to.
        :param library_path_func: function to determine the library path to a
                                  given dependency of ElfFile.
        :param list base_rpaths: library paths for the used base snap.
        """
        self._dynamic_linker = dynamic_linker
        self._library_path_func = library_path_func
        if not base_rpaths:
            base_rpaths = []
        self._base_rpaths = base_rpaths

        # If we are running from the snap we want to use the patchelf
        # bundled there as it would have the capabilty of working
        # anywhere given the fixed ld it would have.
        # If not found, resort to whatever is on the system brought
        # in by packaging dependencies.
        if common.is_snap():
            snap_dir = os.getenv('SNAP')
            self._patchelf_cmd = os.path.join(snap_dir, 'bin', 'patchelf')
        else:
            self._patchelf_cmd = 'patchelf'

    def patch(self, *, elf_file: ElfFile) -> None:
        """Patch elf_file with the Patcher instance configuration.

        If the ELF is executable, patch it to use the configured linker.

        :param ElfFile elf: a data object representing an elf file and its
                            relevant attributes.
        :raises snapcraft.internal.errors.PatcherError:
            raised when the elf_file cannot be patched.
        """
        if elf_file.is_executable:
            self._patch_interpreter(elf_file)
        if elf_file.dependencies:
            self._patch_rpath(elf_file)

    def _patch_interpreter(self, elf_file: ElfFile) -> None:
        self._run_patchelf(['--set-interpreter',  self._dynamic_linker],
                           elf_file.path)

    def _patch_rpath(self, elf_file: ElfFile) -> None:
        rpath = self._get_rpath(elf_file)
        # Parameters:
        # --force-rpath: use RPATH instead of RUNPATH.
        # --shrink-rpath: will remove unneeded entries, with the
        #                 side effect of preferring host libraries
        #                 so we simply do not use it.
        # --set-rpath: set the RPATH to the colon separated argument.
        self._run_patchelf(['--force-rpath',
                            '--set-rpath', rpath],
                           elf_file.path)

    def _run_patchelf(self, args: List[str], elf_file_path: str) -> None:
        try:
            subprocess.check_call([self._patchelf_cmd] + args +
                                  [elf_file_path])
        # There is no need to catch FileNotFoundError as patchelf should be
        # bundled with snapcraft which means its lack of existence is a
        # "packager" error.
        except subprocess.CalledProcessError as call_error:
            raise errors.PatcherError(elf_file=elf_file_path,
                                      message=str(call_error))

    def _get_rpath(self, elf_file) -> str:
        rpaths = []  # type: List[str]
        for dependency in elf_file.dependencies:
            rpaths.append(self._library_path_func(dependency, elf_file.path))

        origin_paths = ':'.join((r for r in set(rpaths) if r))
        base_rpaths = ':'.join(self._base_rpaths)

        return '{}:{}'.format(origin_paths, base_rpaths)


def determine_ld_library_path(root: str) -> List[str]:
    """Determine additional library paths needed for the linker loader.

    This is a workaround until full library searching is implemented which
    works by searching for ld.so.conf in specific hard coded locations
    within root.

    :param root str: the root directory to search for specific ld.so.conf
                     entries.
    :returns: a list of strings of library paths where releavant libraries
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
    if _libraries:
        return _libraries

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


def get_elf_files(root: str,
                  file_list: Sequence[str]) -> FrozenSet[ElfFile]:
    """Return a frozenset of elf files from file_list prepended with root.

    :param str root: the root directory from where the file_list is generated.
    :param file_list: a list of file in root.
    :returns: a frozentset of ElfFile objects.
    """
    ms = magic.open(magic.NONE)
    if ms.load() != 0:
        raise RuntimeError('Cannot load magic header detection')

    elf_files = set()  # type: Set[ElfFile]

    fs_encoding = sys.getfilesystemencoding()

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

        path_b = path.encode(
            fs_encoding, errors='surrogateescape')  # type: bytes
        # Finally, make sure this is actually an ELF before queueing it up
        # for an ldd call.
        file_m = ms.file(path_b)
        if file_m.startswith('ELF') and 'dynamically linked' in file_m:
            is_executable = 'interpreter' in file_m
            elf_files.add(ElfFile(path=path, is_executable=is_executable))

    return frozenset(elf_files)
