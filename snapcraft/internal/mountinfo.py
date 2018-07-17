# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

import collections
import contextlib
import csv
import logging
from typing import Dict, List  # noqa: F401

from snapcraft.internal import errors

logger = logging.getLogger(__name__)


class Mount:
    """A class to provide programmatic access to a specific mountpoint"""

    def __init__(self, mountinfo_row: List[str]) -> None:
        # Parse the row according to section 3.5 of
        # https://www.kernel.org/doc/Documentation/filesystems/proc.txt
        try:
            self.mount_id = mountinfo_row[0]
            self.parent_id = mountinfo_row[1]
            self.st_dev = mountinfo_row[2]
            self.root = mountinfo_row[3]
            self.mount_point = mountinfo_row[4]
            self.mount_options = mountinfo_row[5]
            separator_index = mountinfo_row.index("-")
            self.optional_fields = mountinfo_row[6:separator_index]
            self.filesystem_type = mountinfo_row[separator_index + 1]
            self.mount_source = mountinfo_row[separator_index + 2]
            self.super_options = mountinfo_row[separator_index + 3]
        except IndexError as e:
            raise errors.InvalidMountinfoFormat(" ".join(mountinfo_row)) from e


class MountInfo:
    """A class to provide programmatic access to /proc/self/mountinfo"""

    def __init__(self, *, mountinfo_file: str = "/proc/self/mountinfo") -> None:
        """Create a new MountInfo instance.

        :param str mountinfo_file: Path to mountinfo file to be parsed.
        """
        # Maintain two dicts pointing to the same underlying objects:
        # a dict of mount points to Mounts, and a dict of roots to Mounts.
        self._mount_point_mounts = {}  # type: Dict[str, Mount]
        root_mounts = collections.defaultdict(
            list
        )  # type: Dict[str, List[Mount]]  # noqa

        with contextlib.suppress(FileNotFoundError):
            with open(mountinfo_file) as f:
                for row in csv.reader(f, delimiter=" "):
                    try:
                        mount = Mount(row)
                        self._mount_point_mounts[mount.mount_point] = mount
                        root_mounts[mount.root].append(mount)
                    except errors.InvalidMountinfoFormat as e:
                        logger.warn(str(e))

        self._root_mounts = dict(root_mounts)

    def for_mount_point(self, mount_point: str) -> Mount:
        try:
            return self._mount_point_mounts[mount_point]
        except KeyError:
            raise errors.MountPointNotFoundError(mount_point)

    def for_root(self, root: str) -> List[Mount]:
        try:
            return self._root_mounts[root]
        except KeyError:
            raise errors.RootNotMountedError(root)
