# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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

"""Git repository class and helper utilities."""

import logging
import os
import subprocess
import time
from pathlib import Path
from typing import Optional

import pygit2

from .errors import GitError

logger = logging.getLogger(__name__)


def is_repo(path: Path) -> bool:
    """Check if a directory is a git repo.

    :param path: filepath to check

    :returns: True if path is a git repo.

    :raises GitError: if git fails while checking for a repository
    """
    # `path.absolute().parent` prevents pygit2 from checking parent directories
    try:
        return bool(
            pygit2.discover_repository(str(path), False, str(path.absolute().parent))
        )
    except pygit2.GitError as error:
        raise GitError(
            f"Could not check for git repository in {str(path)!r}."
        ) from error


def is_shallow_repo(path: Path) -> bool:
    """Check if a directory is a shallow cloned git repo.

    :param path: filepath to check

    :returns: True if path is a shallow cloned git repo.

    :raises GitError: if git fails while checking for a repository
    """
    if is_repo(path):
        repo = pygit2.Repository(path)
        return repo.is_shallow

    return False


class GitRepo:
    """Git repository class."""

    def __init__(self, path: Path) -> None:
        """Initialize a git repo.

        If a git repo does not already exist, a new repo will be initialized.

        :param path: filepath of the repo

        :raises FileNotFoundError: if the directory does not exist
        :raises GitError: if the repo cannot be initialized
        """
        self.path = path

        if not path.is_dir():
            raise FileNotFoundError(
                f"Could not initialize a git repository because {str(path)!r} does not "
                "exist or is not a directory."
            )

        if not is_repo(path):
            self._init_repo()

        self._repo = pygit2.Repository(path)

    def add_all(self) -> None:
        """Add all changes from the working tree to the index.

        :raises GitError: if the changes could not be added
        """
        logger.debug("Adding all changes.")

        try:
            self._repo.index.add_all()
            self._repo.index.write()
        except pygit2.GitError as error:
            raise GitError(
                f"Could not add changes for the git repository in {str(self.path)!r}."
            ) from error

    def commit(self, message: str = "auto commit") -> str:
        """Commit changes to the repo.

        :param message: the commit message

        :returns: object ID of the commit as str

        :raises GitError: if the commit could not be created
        """
        logger.debug("Committing changes.")

        try:
            tree = self._repo.index.write_tree()
        except pygit2.GitError as error:
            raise GitError(
                f"Could not create a tree for the git repository in {str(self.path)!r}."
            ) from error

        author = pygit2.Signature("auto commit", "auto commit")

        # a target is not needed for an unborn head (no existing commits in branch)
        target = [] if self._repo.head_is_unborn else [self._repo.head.target]

        try:
            return str(
                self._repo.create_commit("HEAD", author, author, message, tree, target)
            )
        except pygit2.GitError as error:
            raise GitError(
                "Could not create a commit for the git repository "
                f"in {str(self.path)!r}."
            ) from error

    def is_clean(self) -> bool:
        """Check if the repo is clean.

        :returns: True if the repo is clean.

        :raises GitError: if git fails while checking if the repo is clean
        """
        try:
            # for a clean repo, `status()` will return an empty dict
            return not bool(self._repo.status())
        except pygit2.GitError as error:
            raise GitError(
                f"Could not check if the git repository in {str(self.path)!r} is clean."
            ) from error

    def _init_repo(self) -> None:
        """Initialize a git repo.

        :raises GitError: if the repo cannot be initialized
        """
        logger.debug("Initializing git repository in %r", str(self.path))

        try:
            pygit2.init_repository(self.path)
        except pygit2.GitError as error:
            raise GitError(
                f"Could not initialize a git repository in {str(self.path)!r}."
            ) from error

    def push_url(  # pylint: disable=too-many-branches
        self,
        remote_url: str,
        remote_branch: str,
        ref: str = "HEAD",
        token: Optional[str] = None,
        push_tags: bool = False,
    ) -> None:
        """Push a reference to a branch on a remote url.

        :param remote_url: the remote repo URL to push to
        :param remote_branch: the branch on the remote to push to
        :param ref: name of shorthand ref to push (i.e. a branch, tag, or `HEAD`)
        :param token: token in the url to hide in logs and errors
        :param push_tags: if true, push all tags to URL (similar to `git push --tags`)

        :raises GitError: if the ref cannot be resolved or pushed
        """
        resolved_ref = self._resolve_ref(ref)
        refspec = f"{resolved_ref}:refs/heads/{remote_branch}"

        # hide secret tokens embedded in a url
        if token:
            stripped_url = remote_url.replace(token, "<token>")
        else:
            stripped_url = remote_url

        logger.debug(
            "Pushing %r to remote %r with refspec %r.", ref, stripped_url, refspec
        )

        # temporarily call git directly due to libgit2 bug that unable to push
        # large repos using https. See https://github.com/libgit2/libgit2/issues/6385
        # and https://github.com/snapcore/snapcraft/issues/4478
        cmd: list[str] = ["git", "push", remote_url, refspec, "--progress"]
        if push_tags:
            cmd.append("--tags")

        git_proc: Optional[subprocess.Popen] = None
        try:
            with subprocess.Popen(
                cmd,
                cwd=str(self.path),
                bufsize=1,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
            ) as git_proc:
                # do not block on reading from the pipes
                # (has no effect on Windows until Python 3.12, so the readline() method is
                # blocking on Windows but git will still proceed)
                if git_proc.stdout:
                    os.set_blocking(git_proc.stdout.fileno(), False)
                if git_proc.stderr:
                    os.set_blocking(git_proc.stderr.fileno(), False)

                git_stdout: str
                git_stderr: str

                while git_proc.poll() is None:
                    if git_proc.stdout:
                        while git_stdout := git_proc.stdout.readline():
                            logger.info(git_stdout.rstrip())
                    if git_proc.stderr:
                        while git_stderr := git_proc.stderr.readline():
                            logger.error(git_stderr.rstrip())
                    # avoid too much looping, but not too slow to display progress
                    time.sleep(0.01)

        except subprocess.SubprocessError as error:
            # logging the remaining output
            if git_proc:
                if git_proc.stdout:
                    for git_stdout in git_proc.stdout.readlines():
                        logger.info(git_stdout.rstrip())
                if git_proc.stderr:
                    for git_stderr in git_proc.stderr.readlines():
                        logger.error(git_stderr.rstrip())

            raise GitError(
                f"Could not push {ref!r} to {stripped_url!r} with refspec {refspec!r} "
                f"for the git repository in {str(self.path)!r}: "
                f"{error!s}"
            ) from error

        if git_proc:
            git_proc.wait()
            if git_proc.returncode == 0:
                return

        raise GitError(
            f"Could not push {ref!r} to {stripped_url!r} with refspec {refspec!r} "
            f"for the git repository in {str(self.path)!r}."
        )

    def _resolve_ref(self, ref: str) -> str:
        """Get a full reference name for a shorthand ref.

        :param ref: shorthand ref name (i.e. a branch, tag, or `HEAD`)

        :returns: the full ref name (i.e. `refs/heads/main`)

        raises GitError: if the name could not be resolved
        """
        try:
            reference = self._repo.lookup_reference_dwim(ref).name
            logger.debug("Resolved reference %r for name %r", reference, ref)
            return reference
        # raises a KeyError if the ref does not exist and a GitError for git errors
        except (pygit2.GitError, KeyError) as error:
            raise GitError(
                f"Could not resolve reference {ref!r} for the git repository in "
                f"{str(self.path)!r}."
            ) from error
