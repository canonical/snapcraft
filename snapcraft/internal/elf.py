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
import glob
import logging
import os
import re
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


@lru_cache()
def _crawl_for_path(*, soname: str, root_path: str,
                    core_base_path: str) -> str:
    for path in (root_path, core_base_path):
        if not os.path.exists(path):
            continue
        for root, directories, files in os.walk(path):
            for file_name in files:
                if file_name == soname:
                    file_path = os.path.join(root, file_name)
                    if _is_dynamically_linked_elf(_get_magic(file_path)):
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

    def __init__(self, *, dynamic_linker: str, root_path: str) -> None:
        """Create a Patcher instance.

        :param str dynamic_linker: the path to the dynamic linker to set the
                                   elf file to.
        :param str root_path: the base path for the snap to determine
                              if use of $ORIGIN is possible.
        """
        self._dynamic_linker = dynamic_linker
        self._root_path = root_path

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
        origin_rpaths = set()  # type: Set[str]
        base_rpaths = set()  # type: Set[str]
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
                    origin_rpaths.add(rel_library_path_dir.replace(
                        '..', '$ORIGIN', 1))

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


@lru_cache()
def _get_magic(path: str) -> str:
    ms = magic.open(magic.NONE)
    if ms.load() != 0:
        raise RuntimeError('Cannot load magic header detection')

    fs_encoding = sys.getfilesystemencoding()
    path_b = path.encode(
        fs_encoding, errors='surrogateescape')  # type: bytes
    return ms.file(path_b)


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
        file_m = _get_magic(path)
        if _is_dynamically_linked_elf(file_m):
            elf_files.add(ElfFile(path=path, magic=file_m))

    return frozenset(elf_files)
