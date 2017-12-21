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
import logging
import os
import subprocess
import sys
from functools import lru_cache, wraps
from typing import FrozenSet, List, Set, Sequence

import magic

from snapcraft.internal import (
    common,
    errors,
    os_release,
    repo,
)


logger = logging.getLogger(__name__)


class Library:
    """Represents the SONAME and path to the library."""

    def __init__(self, *, soname: str, path: str, base_path: str,
                 core_base_path: str) -> None:
        self.soname = soname
        if path.startswith(base_path) or path.startswith(core_base_path):
            self.path = path
        else:
            self.path = _crawl_for_path(soname=soname,
                                        base_path=base_path,
                                        core_base_path=core_base_path)
            print('crawled for', self.path)

        system_libs = _get_system_libs()
        if soname in system_libs:
            self.system_lib = True
        else:
            self.system_lib = False

        if path.startswith('/snap/'):
            self.in_snap = True
        else:
            self.in_snap = False


@lru_cache()
def _crawl_for_path(*, soname: str, base_path: str,
                    core_base_path: str) -> str:
    for root_path in (base_path, core_base_path):
        if not os.path.exists(root_path):
            continue
        for root, directories, files in os.walk(root_path):
            for file_name in files:
                if file_name == soname:
                    file_path = os.path.join(root, file_name)
                    return file_path
    return None


class ElfFile:
    """ElfFile represents and elf file on a path and its attributes."""

    def __init__(self, *, path: str, magic: str) -> None:
        """Initialize an ElfFile instance.

        :param str path: path to an elf_file within a snapcraft project.
        :param str magic: the magic string for path.
        """
        self.path = path
        self.is_executable = 'interpreter' in magic
        self.dependencies = set()  # type: Set[Library]

    def load_dependencies(self, base_path: str,
                          core_base_path: str) -> Set[str]:
        """Load the set of libraries that are needed to satisfy elf's runtime.

        This may include libraries contained within the project.
        The object's .dependencies attribute is set after loading.

        :param str base_path: the base path to search for missing dependencies.
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
                                 base_path=base_path,
                                 core_base_path=core_base_path))

        self.dependencies = libs

        return {l.path for l in libs
                if l.path and not l.in_snap and not l.system_lib}


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
        # bundled there as it would have the capability of working
        # anywhere given the fixed ld it would have.
        # If not found, resort to whatever is on the system brought
        # in by packaging dependencies.
        # The docker conditional will work if the docker image has the
        # snaps unpacked in the corresponding locations.
        if common.is_snap():
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
        If the ELF has dependencies, set an rpath to them.

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
        self._run_patchelf(args=['--set-interpreter',  self._dynamic_linker],
                           elf_file_path=elf_file.path)

    def _patch_rpath(self, elf_file: ElfFile) -> None:
        rpath = self._get_rpath(elf_file)
        # Parameters:
        # --force-rpath: use RPATH instead of RUNPATH.
        # --shrink-rpath: will remove unneeded entries, with the
        #                 side effect of preferring host libraries
        #                 so we simply do not use it.
        # --set-rpath: set the RPATH to the colon separated argument.
        self._run_patchelf(args=['--force-rpath',
                                 '--set-rpath', rpath],
                           elf_file_path=elf_file.path)

    @_retry_patch
    def _run_patchelf(self, *, args: List[str], elf_file_path: str) -> None:
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
            if dependency.path:
                rpaths.append(self._library_path_func(dependency.path,
                                                      elf_file.path))

        origin_paths = ':'.join((r for r in set(rpaths) if r))
        base_rpaths = ':'.join(self._base_rpaths)

        if origin_paths:
            return '{}:{}'.format(origin_paths, base_rpaths)
        else:
            return base_rpaths


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
            elf_files.add(ElfFile(path=path, magic=file_m))

    return frozenset(elf_files)
