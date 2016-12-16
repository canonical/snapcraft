# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

import re
import glob
import logging
import os
import platform
import subprocess

from snapcraft.internal import common


logger = logging.getLogger(__name__)


def determine_ld_library_path(root):
    # If more ld.so.conf files need to be supported, add them here.
    ld_config_globs = {
        '{}/usr/lib/*/mesa*/ld.so.conf'.format(root)
    }

    ld_library_paths = []
    for this_glob in ld_config_globs:
        for ld_conf_file in glob.glob(this_glob):
            ld_library_paths.extend(_extract_ld_library_paths(ld_conf_file))

    return [root + path for path in ld_library_paths]


def _extract_ld_library_paths(ld_conf_file):
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


def _get_system_libs():
    global _libraries
    if _libraries:
        return _libraries

    release = platform.linux_distribution()[1]
    lib_path = os.path.join(common.get_librariesdir(), release)

    if not os.path.exists(lib_path):
        logger.warning('No libraries to exclude from this release')
        # Always exclude libc.so.6
        return frozenset(['libc.so.6'])

    with open(lib_path) as fn:
        _libraries = frozenset(fn.read().split())

    return _libraries


def get_dependencies(elf):
    """Return a list of libraries that are needed to satisfy elf's runtime.

    This may include libraries contained within the project.
    """
    logger.debug('Getting dependencies for {!r}'.format(elf))
    ldd_out = ''
    try:
        ldd_out = common.run_output(['ldd', elf]).split('\n')
    except subprocess.CalledProcessError:
        logger.warning(
            'Unable to determine library dependencies for {!r}'.format(elf))
        return []
    ldd_out = [l.split() for l in ldd_out]
    ldd_out = [l[2] for l in ldd_out if len(l) > 2 and os.path.exists(l[2])]

    # Now lets filter out what would be on the system
    system_libs = _get_system_libs()
    libs = [l for l in ldd_out if not os.path.basename(l) in system_libs]

    return libs
