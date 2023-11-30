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

    def push_url(
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

        # Create a list of tags to push
        if push_tags:
            tag_refs = [
                t.name
                for t in self._repo.references.iterator(pygit2.GIT_REFERENCES_TAGS)
            ]
        else:
            tag_refs = []
        try:
            self._repo.remotes.create_anonymous(remote_url).push([refspec] + tag_refs)
        except pygit2.GitError as error:
            raise GitError(
                f"Could not push {ref!r} to {stripped_url!r} with refspec {refspec!r} "
                f"for the git repository in {str(self.path)!r}: "
                f"{error!s}"
            ) from error

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
