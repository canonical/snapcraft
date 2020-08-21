# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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

import fileinput
import logging
import os
import re
import stat
from typing import Optional

logger = logging.getLogger(__name__)


def fix_pkg_config(
    root: str, pkg_config_file: str, prefix_trim: Optional[str] = None
) -> None:
    """Opens a pkg_config_file and prefixes the prefix with root."""
    pattern_trim = None
    if prefix_trim:
        pattern_trim = re.compile("^prefix={}(?P<prefix>.*)".format(prefix_trim))
    pattern = re.compile("^prefix=(?P<prefix>.*)")

    with fileinput.input(pkg_config_file, inplace=True) as input_file:
        for line in input_file:
            match = pattern.search(line)
            if prefix_trim is not None and pattern_trim is not None:
                match_trim = pattern_trim.search(line)
            if prefix_trim is not None and match_trim is not None:
                print("prefix={}{}".format(root, match_trim.group("prefix")))
            elif match:
                print("prefix={}{}".format(root, match.group("prefix")))
            else:
                print(line, end="")


def fix_filemode(path: str) -> None:
    mode = stat.S_IMODE(os.stat(path, follow_symlinks=False).st_mode)
    if mode & 0o4000 or mode & 0o2000:
        logger.warning("Removing suid/guid from {}".format(path))
        os.chmod(path, mode & 0o1777)
