# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

import fnmatch
import logging
import os

from pathlib import Path
from snapcraft.file_utils import link_or_copy, link_or_copy_tree
from typing import List
from ._repo import Repo

logger = logging.getLogger(__name__)


class Worktree:
    """Create a copy of the current project directory.

    Worktree keeps the current project directory and a copy synchronized, except
    for a list of control files and architecture-specific binaries. The copy tree
    is maintained under revision control.
    """

    def __init__(self, src: str, dest: str, ignore: List[str] = []) -> None:
        if not os.path.isdir(src):
            raise RuntimeError
        self._srcroot = src
        self._destroot = dest
        self._ignore = [".git", ".gitignore", ".gitmodules", ".bzr", ".svn"]
        self._ignore.extend(ignore)
        Path(dest).mkdir(parents=True, exist_ok=True)
        self._repo = Repo(dest)
        self._repo.reset()

    def sync(self):
        """
        Synchronize the source directory and the work tree.

        Given the path for a source tree, replicate its contents and keep the destination
        in a revision control repository.
        """

        # add files to work tree
        link_or_copy_tree(
            source_tree=self._srcroot,
            destination_tree=self._destroot,
            ignore=self._ignored_files,
            copy_function=self._link_and_add,
        )

        git_dir = os.path.join(self._destroot, ".git")

        # now remove files that exist only in destination
        for dirpath, dirnames, filenames in os.walk(self._destroot, topdown=False):
            if dirpath.startswith(git_dir):
                continue
            dirnames = [d for d in dirnames if d != ".git"]

            for name in filenames:
                dest = os.path.join(dirpath, name)
                src = os.path.join(self._srcpath(dirpath), name)
                if not os.path.exists(src):
                    logger.debug("Remove file: {}".format(dest))
                    self._unlink(dest)

            for name in dirnames:
                dest = os.path.join(dirpath, name)
                src = os.path.join(self._srcpath(dirpath), name)
                if not os.path.exists(src) and os.path.exists(dest):
                    logger.debug("Remove directory: {}".format(dest))
                    os.rmdir(dest)

        # finally commit our changes, if any
        self._repo.commit()

    def add_remote(self, provider: str, user: str, build_id: str) -> str:
        """Add a remote repository."""
        return self._repo.add_remote(provider, user, build_id)

    def push(self) -> None:
        """Send our data to a remote repository."""
        self._repo.push()

    def _ignored_files(self, source: str, contents: List[str]) -> List[str]:
        ignored = set()
        for pattern in self._ignore:
            ignored.update(set(fnmatch.filter(contents, pattern)))
        return list(ignored)

    def _link_and_add(self, source: str, destination: str) -> None:
        link_or_copy(source, destination)
        self._repo.add(os.path.relpath(destination, self._destroot))
        logger.debug("Add file {}".format(destination))

    def _unlink(self, path: str) -> None:
        self._repo.remove(os.path.relpath(path, self._destroot))

    def _srcpath(self, path: str) -> str:
        rel = os.path.relpath(path, self._destroot)
        return os.path.join(self._srcroot, rel)
