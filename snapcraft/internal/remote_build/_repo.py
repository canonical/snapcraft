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

import importlib

from typing import List
from typing import Any  # noqa: F401
from . import errors

import snapcraft


class Repo:
    """Project repository management operations."""

    def __init__(self, path: str) -> None:
        self._root = path

        try:
            git = importlib.import_module("git")  # type: Any
        except ImportError:
            raise errors.GitNotFoundError

        try:
            self._repo = git.Repo(path)
        except git.exc.InvalidGitRepositoryError:
            self._repo = git.Repo.init(path)

    def add(self, entry: str) -> None:
        self._repo.index.add([str(entry)])

    def remove(self, entry: str) -> None:
        self._repo.index.remove([str(entry)], working_tree=True)

    def commit(self) -> None:
        if self._repo.is_dirty():
            self._repo.index.commit(
                "snapcraft commit\n\nversion: {}".format(snapcraft.__version__)
            )

    def push_remote(self, provider: str, user: str, branch: str, build_id: str) -> str:
        """Push without adding a remote to the local repository."""

        if provider == "launchpad":
            url = self._remote_url(user, build_id)
            self._repo.git.push(url, branch, force=True)
            self._repo.git.push(url, "--tags")
        else:
            raise errors.RemoteBuilderNotSupportedError(provider=provider)
        return url

    def add_remote(self, provider: str, user: str, build_id: str) -> str:
        if provider == "launchpad":
            url = self._remote_url(user, build_id)
        else:
            raise errors.RemoteBuilderNotSupportedError(provider=provider)

        for remote in self._repo.remotes:
            if remote.name == provider:
                self._remote = remote
                return url

        self._remote = self._repo.create_remote(provider, url=url)
        return url

    def push(self):
        self._remote.push(refspec="master:master")

    def reset(self) -> None:
        self._repo.git.reset("--hard")

    @property
    def is_dirty(self) -> bool:
        return self._repo.is_dirty()

    @property
    def uncommitted_files(self) -> List[str]:
        files = []
        for item in self._repo.index.diff(None):
            files.append(item.a_path)
        return files

    @property
    def branch_name(self) -> str:
        return self._repo.active_branch.name

    @staticmethod
    def _remote_url(user: str, build_id: str) -> str:
        # TODO: change this after launchpad infrastructure is ready
        url = "git+ssh://{user}@git.launchpad.net/~{user}/+git/{id}/".format(
            user=user, id=build_id
        )
        return url
