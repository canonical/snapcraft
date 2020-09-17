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

import abc
import logging
import re
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, List, Optional

from snapcraft.internal import repo

logger = logging.getLogger(__name__)


class PackageRepository(abc.ABC):
    @abc.abstractmethod
    def install(self, *, keys_path: Path) -> bool:
        ...

    @abc.abstractmethod
    def marshal(self) -> Dict[str, Any]:
        ...

    @classmethod
    def unmarshal(cls, data: Dict[str, str]) -> "PackageRepository":
        if not isinstance(data, dict):
            raise RuntimeError(f"invalid repository object: {data!r}")

        if "ppa" in data:
            return PackageRepositoryAptPpa.unmarshal(data)

        return PackageRepositoryApt.unmarshal(data)

    @classmethod
    def unmarshal_package_repositories(cls, data: Any) -> List["PackageRepository"]:
        repositories = list()

        if data is not None:
            if not isinstance(data, list):
                raise RuntimeError(f"invalid package-repositories: {data!r}")

            for repository in data:
                package_repo = cls.unmarshal(repository)
                repositories.append(package_repo)

        return repositories


class PackageRepositoryAptPpa(PackageRepository):
    def __init__(self, *, ppa: str) -> None:
        self.type = "apt"
        self.ppa = ppa

    def install(self, *, keys_path: Path) -> bool:
        return repo.Ubuntu.install_ppa(keys_path=keys_path, ppa=self.ppa)

    def marshal(self) -> Dict[str, Any]:
        data = dict(type="apt")
        data["ppa"] = self.ppa
        return data

    @classmethod
    def unmarshal(cls, data: Dict[str, str]) -> "PackageRepositoryAptPpa":
        if not isinstance(data, dict):
            raise RuntimeError(f"invalid ppa repository object: {data!r}")

        data_copy = deepcopy(data)

        ppa = data_copy.pop("ppa", None)
        repo_type = data_copy.pop("type", None)
        if repo_type != "apt":
            raise RuntimeError(f"invalid ppa repository object {data!r} (type invalid)")

        if ppa is None:
            raise RuntimeError(f"invalid ppa repository object {data!r} (ppa missing)")

        if data_copy:
            raise RuntimeError(f"invalid ppa repository object {data!r} (extra keys)")

        return cls(ppa=ppa)


class PackageRepositoryApt(PackageRepository):
    def __init__(
        self,
        *,
        architectures: Optional[List[str]] = None,
        components: List[str],
        deb_types: Optional[List[str]] = None,
        key_id: str,
        key_server: Optional[str] = None,
        name: Optional[str] = None,
        suites: List[str],
        url: str,
    ) -> None:
        self.type = "apt"
        self.architectures = architectures
        self.components = components
        self.deb_types = deb_types
        self.key_id = key_id
        self.key_server = key_server

        if name is None:
            # Default name is URL, stripping non-alphanumeric characters.
            self.name: str = re.sub(r"\W+", "_", url)
        else:
            self.name = name

        self.suites = suites
        self.url = url

    def install(self, keys_path: Path) -> bool:
        # First install associated GPG key.
        new_key: bool = repo.Ubuntu.install_gpg_key_id(
            keys_path=keys_path, key_id=self.key_id, key_server=self.key_server
        )

        # Now install sources file.
        new_sources: bool = repo.Ubuntu.install_sources(
            architectures=self.architectures,
            components=self.components,
            deb_types=self.deb_types,
            name=self.name,
            suites=self.suites,
            url=self.url,
        )

        return new_key or new_sources

    def marshal(self) -> Dict[str, Any]:
        data: Dict[str, Any] = {"type": "apt"}

        if self.architectures:
            data["architectures"] = self.architectures

        data["components"] = self.components

        if self.deb_types:
            data["formats"] = self.deb_types

        data["key-id"] = self.key_id

        if self.key_server:
            data["key-server"] = self.key_server

        data["name"] = self.name
        data["suites"] = self.suites
        data["url"] = self.url

        return data

    @classmethod  # noqa: C901
    def unmarshal(cls, data: Dict[str, Any]) -> "PackageRepositoryApt":
        if not isinstance(data, dict):
            raise RuntimeError(f"invalid deb repository object: {data!r}")

        data_copy = deepcopy(data)

        architectures = data_copy.pop("architectures", None)
        components = data_copy.pop("components", None)
        deb_types = data_copy.pop("formats", None)
        key_id = data_copy.pop("key-id", None)
        key_server = data_copy.pop("key-server", None)
        name = data_copy.pop("name", None)
        suites = data_copy.pop("suites", None)
        url = data_copy.pop("url", None)
        repo_type = data_copy.pop("type", None)

        if repo_type != "apt":
            raise RuntimeError(
                f"invalid deb repository object: {data!r} (invalid type)"
            )

        if architectures is not None and (
            not isinstance(architectures, list)
            or not all(isinstance(x, str) for x in architectures)
        ):
            raise RuntimeError(
                f"invalid deb repository object: {data!r} (invalid architectures)"
            )

        if not isinstance(components, list) or not all(
            isinstance(x, str) for x in components
        ):
            raise RuntimeError(
                f"invalid deb repository object: {data!r} (invalid components)"
            )

        if deb_types is not None and any(
            deb_type not in ["deb", "deb-src"] for deb_type in deb_types
        ):
            raise RuntimeError(
                f"invalid deb repository object: {data!r} (invalid formats)"
            )

        if not isinstance(key_id, str):
            raise RuntimeError(
                f"invalid deb repository object: {data!r} (invalid key-id)"
            )

        if key_server is not None and not isinstance(key_server, str):
            raise RuntimeError(
                f"invalid deb repository object: {data!r} (invalid key-server)"
            )

        if name is not None and not isinstance(name, str):
            raise RuntimeError(
                f"invalid deb repository object: {data!r} (invalid name)"
            )

        if not isinstance(suites, list) or not all(isinstance(x, str) for x in suites):
            raise RuntimeError(
                f"invalid deb repository object: {data!r} (invalid suites)"
            )

        if not isinstance(url, str):
            raise RuntimeError(f"invalid deb repository object: {data!r} (invalid url)")

        if data_copy:
            raise RuntimeError(f"invalid deb repository object: {data!r} (extra keys)")

        return cls(
            architectures=architectures,
            components=components,
            deb_types=deb_types,
            key_id=key_id,
            key_server=key_server,
            name=name,
            suites=suites,
            url=url,
        )
