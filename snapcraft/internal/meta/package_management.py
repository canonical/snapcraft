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

import logging
from collections import OrderedDict
from typing import Any, Dict, List, Optional

from snapcraft.internal.meta import errors

logger = logging.getLogger(__name__)


class Repository:
    def __init__(
        self,
        *,
        source: str,
        gpg_public_key: Optional[str] = None,
        gpg_public_key_id: Optional[str] = None,
        gpg_key_server: Optional[str] = None,
    ) -> None:
        self.source = source
        self.gpg_public_key = gpg_public_key
        self.gpg_public_key_id = gpg_public_key_id
        self.gpg_key_server = gpg_key_server

    @classmethod
    def from_dict(cls, repo_dict: Dict[str, str]) -> "Repository":
        source = repo_dict.get("source")
        if not source:
            raise errors.RepositoryValidationError(
                message=f"'source' undefined for {repo_dict!r}"
            )

        gpg_public_key = repo_dict.get("gpg-public-key")
        gpg_public_key_id = repo_dict.get("gpg-public-key-id")
        gpg_key_server = repo_dict.get("gpg-key-server")

        return Repository(
            source=source,
            gpg_public_key=gpg_public_key,
            gpg_public_key_id=gpg_public_key_id,
            gpg_key_server=gpg_key_server,
        )

    @classmethod
    def from_object(cls, repo_object: Any) -> "Repository":
        if repo_object is None:
            raise errors.RepositoryValidationError(message=f"empty definition")

        if not isinstance(repo_object, dict):
            raise errors.RepositoryValidationError(
                message=f"unknown syntax for {repo_object!r}"
            )

        return cls.from_dict(repo_object)

    def to_dict(self):
        repo_dict = OrderedDict()
        repo_dict["source"] = self.source
        repo_dict["gpg-public-key"] = self.gpg_public_key
        repo_dict["gpg-public-key-id"] = self.gpg_public_key_id
        repo_dict["gpg-key-server"] = self.gpg_key_server
        return repo_dict

    def validate(self):
        if not self.source:
            raise errors.RepositoryValidationError(
                message=f"invalid source '{self.source!r}"
            )

    def __repr__(self):
        return f"Repository(source={self.source!r}, gpg-public-key={self.gpg_public_key}, gpg-public-key-id={self.gpg_public_key_id!r}, gpg-key-server={self.gpg_key_server})"

    def __eq__(self, other: Any) -> bool:
        return repr(self) == repr(other)


class PackageManagement:
    def __init__(self, *, repositories: Optional[List[Repository]] = None):
        if repositories is None:
            self.repositories: List[Repository] = list()
        else:
            self.repositories = repositories

    @classmethod
    def from_dict(cls, pkg_management_dict: Dict[str, Any]) -> "PackageManagement":
        pm = PackageManagement()

        raw_repositories = pkg_management_dict.get("repositories", [])
        for raw_repo in raw_repositories:
            repo = Repository.from_object(raw_repo)
            pm.repositories.append(repo)

        return pm

    @classmethod
    def from_object(cls, pkg_management_object: Any) -> "PackageManagement":
        if pkg_management_object is None:
            return PackageManagement()

        if not isinstance(pkg_management_object, dict):
            raise errors.PackageManagementValidationError(
                message=f"unknown syntax for {pkg_management_object!r}"
            )

        return cls.from_dict(pkg_management_object)

    def validate(self):
        for repo in self.repositories:
            repo.validate()
