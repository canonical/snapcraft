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
from typing import List, Set, Sequence, FrozenSet

import magic

from snapcraft.internal import (
    common,
    errors,
    os_release,
    repo,
)


logger = logging.getLogger(__name__)


def determine_ld_library_path(root: str) -> List[str]:
    """Determine additional library paths needed for the linker loader.

    This is a workaround until full library searching is implemented which
    works by searching for ld.so.conf in specific hard coded locations
    within root.

    :param root str: the root directory to search for specific ld.so.conf
                     entries.
    :returns: a list of strings of library paths where releavant libraries
              can be found withing root.
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


def get_dependencies(elf: str) -> Set[str]:
    """Return a set of libraries that are needed to satisfy elf's runtime.

    This may include libraries contained within the project.

    :param str elf: the elf file to resolve dependencies for.
    :returns: a set of string with paths to the library dependencies of elf.
    """
    logger.debug('Getting dependencies for {!r}'.format(str(elf)))
    ldd_out = []  # type: List[str]
    try:
        ldd_out = common.run_output(['ldd', elf]).split('\n')
    except subprocess.CalledProcessError:
        logger.warning(
            'Unable to determine library dependencies for {!r}'.format(elf))
        return set()
    ldd_out_split = [l.split() for l in ldd_out]
    ldd_out_list = [l[2] for l in ldd_out_split
                    if len(l) > 2 and os.path.exists(l[2])]

    # Now lets filter out what would be on the system
    system_libs = _get_system_libs()
    libs = {l for l in ldd_out_list if not os.path.basename(l) in system_libs}
    # Classic confinement won't do what you want with library crawling
    # and has a nasty side effect described in LP: #1670100
    libs = {l for l in libs if not l.startswith('/snap/')}

    return libs


def get_elf_files(root: str, file_list: Sequence[str]) -> FrozenSet[str]:
    """Return a frozenset of elf files from file_list prepended with root.

    :param str root: the root directory from where the file_list is generated.
    :param file_list: a list of file in root.
    :returns: a frozentset of strings representing paths of valid elf files
              found in root from file_list.
    """
    ms = magic.open(magic.NONE)
    if ms.load() != 0:
        raise RuntimeError('Cannot load magic header detection')

    elf_files = set()

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
            elf_files.add(path)

    return frozenset(elf_files)
