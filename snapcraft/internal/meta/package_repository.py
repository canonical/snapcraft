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

from copy import deepcopy
import logging
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)


class PackageRepository:
    @classmethod
    def unmarshal(cls, data: Dict[str, str]) -> "PackageRepository":
        if not isinstance(data, dict):
            raise RuntimeError(f"invalid repository object: {data!r}")

        if "ppa" in data:
            return PackageRepositoryAptPpa.unmarshal(data)

        return PackageRepositoryAptDeb.unmarshal(data)

    @classmethod
    def unmarshal_package_repositories(cls, data: Any) -> List["PackageRepository"]:
        repositories = list()

        if data is not None:
            if not isinstance(data, list):
                raise RuntimeError(f"invalid package-repositories: {data!r}")

            for repo in data:
                package_repo = cls.unmarshal(repo)
                repositories.append(package_repo)

        return repositories


class PackageRepositoryAptPpa(PackageRepository):
    def __init__(self, *, ppa: str) -> None:
        self.type = "apt"
        self.ppa = ppa

    def marshal(self):
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


class PackageRepositoryAptDeb(PackageRepository):
    def __init__(
        self,
        *,
        architectures: Optional[List[str]] = None,
        components: List[str],
        deb_types: Optional[List[str]] = None,
        key_id: str,
        key_server: Optional[str] = None,
        name: str,
        suites: List[str],
        url: str,
    ) -> None:
        self.type = "apt"
        self.architectures = architectures
        self.components = components
        self.deb_types = deb_types
        self.key_id = key_id
        self.key_server = key_server
        self.name = name
        self.suites = suites
        self.url = url

    def marshal(self):
        data = {"type": "apt"}

        if self.architectures:
            data["architectures"] = self.architectures

        data["components"] = self.components

        if self.deb_types:
            data["deb-types"] = self.deb_types

        data["key-id"] = self.key_id

        if self.key_server:
            data["key-server"] = self.key_server

        data["name"] = self.name
        data["suites"] = self.suites
        data["url"] = self.url

        return data

    @classmethod  # noqa: C901
    def unmarshal(cls, data: Dict[str, Any]) -> "PackageRepositoryAptDeb":
        if not isinstance(data, dict):
            raise RuntimeError(f"invalid deb repository object: {data!r}")

        data_copy = deepcopy(data)

        architectures = data_copy.pop("architectures", None)
        components = data_copy.pop("components", None)
        deb_types = data_copy.pop("deb-types", None)
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
                f"invalid deb repository object: {data!r} (invalid deb-types)"
            )

        if not isinstance(key_id, str):
            raise RuntimeError(
                f"invalid deb repository object: {data!r} (invalid key-id)"
            )

        if key_server is not None and not isinstance(key_server, str):
            raise RuntimeError(
                f"invalid deb repository object: {data!r} (invalid key-server)"
            )

        if not isinstance(name, str):
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
