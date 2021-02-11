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
from typing import Any, Dict, List, Optional

from . import errors

logger = logging.getLogger(__name__)


class PackageRepository(abc.ABC):
    @abc.abstractmethod
    def marshal(self) -> Dict[str, Any]:
        ...

    @classmethod
    def unmarshal(cls, data: Dict[str, str]) -> "PackageRepository":
        if not isinstance(data, dict):
            raise errors.PackageRepositoryValidationError(
                url=str(data),
                brief=f"Invalid package repository object {data!r}.",
                details="Package repository must be a valid dictionary object.",
                resolution="Verify repository configuration and ensure that the correct syntax is used.",
            )

        if "ppa" in data:
            return PackageRepositoryAptPpa.unmarshal(data)

        return PackageRepositoryApt.unmarshal(data)

    @classmethod
    def unmarshal_package_repositories(cls, data: Any) -> List["PackageRepository"]:
        repositories = list()

        if data is not None:
            if not isinstance(data, list):
                raise errors.PackageRepositoryValidationError(
                    url=str(data),
                    brief=f"Invalid package-repositories list object {data!r}.",
                    details="Package repositories must be a list of objects.",
                    resolution="Verify 'package-repositories' configuration and ensure that the correct syntax is used.",
                )

            for repository in data:
                package_repo = cls.unmarshal(repository)
                repositories.append(package_repo)

        return repositories


class PackageRepositoryAptPpa(PackageRepository):
    def __init__(self, *, ppa: str) -> None:
        self.type = "apt"
        self.ppa = ppa

        self.validate()

    def marshal(self) -> Dict[str, Any]:
        data = dict(type="apt")
        data["ppa"] = self.ppa
        return data

    def validate(self) -> None:
        if not self.ppa:
            raise errors.PackageRepositoryValidationError(
                url=self.ppa,
                brief=f"Invalid PPA {self.ppa!r}.",
                details="PPAs must be non-empty strings.",
                resolution="Verify repository configuration and ensure that 'ppa' is correctly specified.",
            )

    @classmethod
    def unmarshal(cls, data: Dict[str, str]) -> "PackageRepositoryAptPpa":
        if not isinstance(data, dict):
            raise errors.PackageRepositoryValidationError(
                url=str(data),
                brief=f"Invalid package repository object {data!r}.",
                details="Package repository must be a valid dictionary object.",
                resolution="Verify repository configuration and ensure that the correct syntax is used.",
            )

        data_copy = deepcopy(data)

        ppa = data_copy.pop("ppa", "")
        repo_type = data_copy.pop("type", None)

        if repo_type != "apt":
            raise errors.PackageRepositoryValidationError(
                url=ppa,
                brief=f"Unsupported type {repo_type!r}.",
                details="The only currently supported type is 'apt'.",
                resolution="Verify repository configuration and ensure that 'type' is correctly specified.",
            )

        if not isinstance(ppa, str):
            raise errors.PackageRepositoryValidationError(
                url=ppa,
                brief=f"Invalid PPA {ppa!r}.",
                details="PPA must be a valid string.",
                resolution="Verify repository configuration and ensure that 'ppa' is correctly specified.",
            )

        if data_copy:
            keys = ", ".join([repr(k) for k in data_copy.keys()])
            raise errors.PackageRepositoryValidationError(
                url=ppa,
                brief=f"Found unsupported package repository properties {keys}.",
                resolution="Verify repository configuration and ensure that it is correct.",
            )

        return cls(ppa=ppa)


class PackageRepositoryApt(PackageRepository):
    def __init__(
        self,
        *,
        architectures: Optional[List[str]] = None,
        components: Optional[List[str]] = None,
        formats: Optional[List[str]] = None,
        key_id: str,
        key_server: Optional[str] = None,
        name: Optional[str] = None,
        path: Optional[str] = None,
        suites: Optional[List[str]] = None,
        url: str,
    ) -> None:
        self.type = "apt"
        self.architectures = architectures
        self.components = components
        self.formats = formats
        self.key_id = key_id
        self.key_server = key_server

        if name is None:
            # Default name is URL, stripping non-alphanumeric characters.
            self.name: str = re.sub(r"\W+", "_", url)
        else:
            self.name = name

        self.path = path
        self.suites = suites
        self.url = url

        self.validate()

    def marshal(self) -> Dict[str, Any]:
        data: Dict[str, Any] = {"type": "apt"}

        if self.architectures:
            data["architectures"] = self.architectures

        if self.components:
            data["components"] = self.components

        if self.formats:
            data["formats"] = self.formats

        data["key-id"] = self.key_id

        if self.key_server:
            data["key-server"] = self.key_server

        data["name"] = self.name

        if self.path:
            data["path"] = self.path

        if self.suites:
            data["suites"] = self.suites

        data["url"] = self.url

        return data

    def validate(self) -> None:  # noqa: C901
        if self.formats is not None:
            for repo_format in self.formats:
                if repo_format not in ["deb", "deb-src"]:
                    raise errors.PackageRepositoryValidationError(
                        url=self.url,
                        brief=f"Invalid format {repo_format!r}.",
                        details="Valid formats include: deb and deb-src.",
                        resolution="Verify the repository configuration and ensure that 'formats' is correctly specified.",
                    )

        if not self.key_id or not re.match(r"^[0-9A-F]{40}$", self.key_id):
            raise errors.PackageRepositoryValidationError(
                url=self.url,
                brief=f"Invalid key identifier {self.key_id!r}.",
                details="Key IDs must be 40 upper-case hex characters.",
                resolution="Verify the repository configuration and ensure that 'key-id' is correctly specified.",
            )

        if not self.url:
            raise errors.PackageRepositoryValidationError(
                url=self.url,
                brief=f"Invalid URL {self.url!r}.",
                details="URLs must be non-empty strings.",
                resolution="Verify the repository configuration and ensure that 'url' is correctly specified.",
            )

        if self.suites:
            for suite in self.suites:
                if suite.endswith("/"):
                    raise errors.PackageRepositoryValidationError(
                        url=self.url,
                        brief=f"Invalid suite {suite!r}.",
                        details="Suites must not end with a '/'.",
                        resolution="Verify the repository configuration and remove the trailing '/ from suites or use the 'path' property to define a path.",
                    )

        if self.path is not None and self.path == "":
            raise errors.PackageRepositoryValidationError(
                url=self.url,
                brief=f"Invalid path {self.path!r}.",
                details="Paths must be non-empty strings.",
                resolution="Verify the repository configuration and ensure that 'path' is a non-empty string such as '/'.",
            )

        if self.path and self.components:
            raise errors.PackageRepositoryValidationError(
                url=self.url,
                brief=f"Components {self.components!r} cannot be combined with path {self.path!r}.",
                details="Path and components are incomptiable options.",
                resolution="Verify the repository configuration and remove 'path' or 'components'.",
            )

        if self.path and self.suites:
            raise errors.PackageRepositoryValidationError(
                url=self.url,
                brief=f"Suites {self.suites!r} cannot be combined with path {self.path!r}.",
                details="Path and suites are incomptiable options.",
                resolution="Verify the repository configuration and remove 'path' or 'suites'.",
            )

        if self.suites and not self.components:
            raise errors.PackageRepositoryValidationError(
                url=self.url,
                brief=f"No components specified.",
                details="Components are required when using suites.",
                resolution="Verify the repository configuration and ensure that 'components' is correctly specified.",
            )

        if self.components and not self.suites:
            raise errors.PackageRepositoryValidationError(
                url=self.url,
                brief=f"No suites specified.",
                details="Suites are required when using components.",
                resolution="Verify the repository configuration and ensure that 'suites' is correctly specified.",
            )

    @classmethod  # noqa: C901
    def unmarshal(cls, data: Dict[str, Any]) -> "PackageRepositoryApt":
        if not isinstance(data, dict):
            raise errors.PackageRepositoryValidationError(
                url=str(data),
                brief=f"Invalid package repository object {data!r}.",
                details="Package repository must be a valid dictionary object.",
                resolution="Verify repository configuration and ensure that the correct syntax is used.",
            )

        data_copy = deepcopy(data)

        architectures = data_copy.pop("architectures", None)
        components = data_copy.pop("components", None)
        formats = data_copy.pop("formats", None)
        key_id = data_copy.pop("key-id", None)
        key_server = data_copy.pop("key-server", None)
        name = data_copy.pop("name", None)
        path = data_copy.pop("path", None)
        suites = data_copy.pop("suites", None)
        url = data_copy.pop("url", "")
        repo_type = data_copy.pop("type", None)

        if repo_type != "apt":
            raise errors.PackageRepositoryValidationError(
                url=url,
                brief=f"Unsupported type {repo_type!r}.",
                details="The only currently supported type is 'apt'.",
                resolution="Verify repository configuration and ensure that 'type' is correctly specified.",
            )

        if architectures is not None and (
            not isinstance(architectures, list)
            or not all(isinstance(x, str) for x in architectures)
        ):
            raise errors.PackageRepositoryValidationError(
                url=url,
                brief=f"Invalid architectures {architectures!r}.",
                details="Architectures must be a list of valid architecture strings.",
                resolution="Verify repository configuration and ensure that 'architectures' is correctly specified.",
            )

        if components is not None and (
            not isinstance(components, list)
            or not all(isinstance(x, str) for x in components)
            or not components
        ):
            raise errors.PackageRepositoryValidationError(
                url=url,
                brief=f"Invalid components {components!r}.",
                details="Components must be a list of strings.",
                resolution="Verify repository configuration and ensure that 'components' is correctly specified.",
            )

        if formats is not None and (
            not isinstance(formats, list)
            or not all(isinstance(x, str) for x in formats)
        ):
            raise errors.PackageRepositoryValidationError(
                url=url,
                brief=f"Invalid formats {formats!r}.",
                details="Formats must be a list of strings.",
                resolution="Verify repository configuration and ensure that 'formats' is correctly specified.",
            )

        if not isinstance(key_id, str):
            raise errors.PackageRepositoryValidationError(
                url=url,
                brief=f"Invalid key identifier {key_id!r}.",
                details="Key identifiers must be a valid string.",
                resolution="Verify repository configuration and ensure that 'key-id' is correctly specified.",
            )

        if key_server is not None and not isinstance(key_server, str):
            raise errors.PackageRepositoryValidationError(
                url=url,
                brief=f"Invalid key server {key_server!r}.",
                details="Key servers must be a valid string.",
                resolution="Verify repository configuration and ensure that 'key-server' is correctly specified.",
            )

        if name is not None and not isinstance(name, str):
            raise errors.PackageRepositoryValidationError(
                url=url,
                brief=f"Invalid name {name!r}.",
                details="Names must be a valid string.",
                resolution="Verify repository configuration and ensure that 'name' is correctly specified.",
            )

        if path is not None and not isinstance(path, str):
            raise errors.PackageRepositoryValidationError(
                url=url,
                brief=f"Invalid path {path!r}.",
                details="Paths must be a valid string.",
                resolution="Verify repository configuration and ensure that 'path' is correctly specified.",
            )

        if suites is not None and (
            not isinstance(suites, list)
            or not all(isinstance(x, str) for x in suites)
            or not suites
        ):
            raise errors.PackageRepositoryValidationError(
                url=url,
                brief=f"Invalid suites {suites!r}.",
                details="Suites must be a list of strings.",
                resolution="Verify repository configuration and ensure that 'suites' is correctly specified.",
            )

        if not isinstance(url, str):
            raise errors.PackageRepositoryValidationError(
                url=url,
                brief=f"Invalid URL {url!r}.",
                details="URLs must be a valid string.",
                resolution="Verify repository configuration and ensure that 'url' is correctly specified.",
            )

        if data_copy:
            keys = ", ".join([repr(k) for k in data_copy.keys()])
            raise errors.PackageRepositoryValidationError(
                url=url,
                brief=f"Found unsupported package repository properties {keys}.",
                resolution="Verify repository configuration and ensure it is correct.",
            )

        return cls(
            architectures=architectures,
            components=components,
            formats=formats,
            key_id=key_id,
            key_server=key_server,
            name=name,
            suites=suites,
            url=url,
        )
