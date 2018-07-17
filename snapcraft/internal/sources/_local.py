# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2016 Canonical Ltd
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

import copy
import functools
import glob
import os

from snapcraft import file_utils
from snapcraft.internal import common
from ._base import Base


class Local(Base):
    def __init__(self, *args, copy_function=file_utils.link_or_copy, **kwargs):
        super().__init__(*args, **kwargs)
        self.source_abspath = os.path.abspath(self.source)
        self.copy_function = copy_function

        self._ignore = functools.partial(_ignore, self.source_abspath, os.getcwd())

    def pull(self):
        file_utils.link_or_copy_tree(
            self.source_abspath,
            self.source_dir,
            ignore=self._ignore,
            copy_function=self.copy_function,
        )

    def _check(self, target):
        try:
            target_mtime = os.lstat(target).st_mtime
        except FileNotFoundError:
            return False

        self._updated_files = set()
        self._updated_directories = set()

        for (root, directories, files) in os.walk(self.source_abspath, topdown=True):
            ignored = set(self._ignore(root, directories + files))
            if ignored:
                # Prune our search appropriately given an ignore list, i.e.
                # don't walk into directories that are ignored.
                directories[:] = [d for d in directories if d not in ignored]

            for file_name in set(files) - ignored:
                path = os.path.join(root, file_name)
                if os.lstat(path).st_mtime >= target_mtime:
                    self._updated_files.add(os.path.relpath(path, self.source))

            for directory in directories:
                path = os.path.join(root, directory)
                if os.lstat(path).st_mtime >= target_mtime:
                    # Don't decend into this directory-- we'll just copy it
                    # entirely.
                    directories.remove(directory)

                    # os.walk will include symlinks to directories here, but we
                    # want to treat those as files
                    relpath = os.path.relpath(path, self.source)
                    if os.path.islink(path):
                        self._updated_files.add(relpath)
                    else:
                        self._updated_directories.add(relpath)

        return len(self._updated_files) > 0 or len(self._updated_directories) > 0

    def _update(self):
        # First, copy the directories
        for directory in self._updated_directories:
            file_utils.link_or_copy_tree(
                os.path.join(self.source, directory),
                os.path.join(self.source_dir, directory),
                ignore=self._ignore,
                copy_function=self.copy_function,
            )

        # Now, copy files
        for file_path in self._updated_files:
            self.copy_function(
                os.path.join(self.source, file_path),
                os.path.join(self.source_dir, file_path),
            )


def _ignore(source, current_directory, directory, files):
    if directory == source or directory == current_directory:
        ignored = copy.copy(common.SNAPCRAFT_FILES)
        snaps = glob.glob(os.path.join(directory, "*.snap"))
        if snaps:
            snaps = [os.path.basename(s) for s in snaps]
            ignored += snaps
        return ignored
    else:
        return []
