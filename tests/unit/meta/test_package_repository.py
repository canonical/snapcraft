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


import pytest

from snapcraft.internal.meta import errors
from snapcraft.internal.meta.package_repository import (
    PackageRepository,
    PackageRepositoryApt,
    PackageRepositoryAptPpa,
)


def test_apt_name():
    repo = PackageRepositoryApt(
        architectures=["amd64", "i386"],
        components=["main", "multiverse"],
        formats=["deb", "deb-src"],
        key_id="A" * 40,
        key_server="keyserver.ubuntu.com",
        suites=["xenial", "xenial-updates"],
        url="http://archive.ubuntu.com/ubuntu",
    )

    assert repo.name == "http_archive_ubuntu_com_ubuntu"


@pytest.mark.parametrize(
    "arch", ["amd64", "armhf", "arm64", "i386", "ppc64el", "riscv", "s390x"]
)
def test_apt_valid_architectures(arch):
    package_repo = PackageRepositoryApt(
        key_id="A" * 40, url="http://test", architectures=[arch]
    )

    assert package_repo.architectures == [arch]


def test_apt_invalid_url():
    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryApt(
            key_id="A" * 40, url="",
        )

    assert exc_info.value.brief == "Invalid URL ''."
    assert exc_info.value.details == "URLs must be non-empty strings."
    assert (
        exc_info.value.resolution
        == "Verify the repository configuration and ensure that 'url' is correctly specified."
    )


def test_apt_invalid_path():
    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryApt(
            key_id="A" * 40, path="", url="http://archive.ubuntu.com/ubuntu",
        )

    assert exc_info.value.brief == "Invalid path ''."
    assert exc_info.value.details == "Paths must be non-empty strings."
    assert (
        exc_info.value.resolution
        == "Verify the repository configuration and ensure that 'path' is a non-empty string such as '/'."
    )


def test_apt_invalid_path_with_suites():
    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryApt(
            key_id="A" * 40,
            path="/",
            suites=["xenial", "xenial-updates"],
            url="http://archive.ubuntu.com/ubuntu",
        )

    assert (
        exc_info.value.brief
        == "Suites ['xenial', 'xenial-updates'] cannot be combined with path '/'."
    )
    assert exc_info.value.details == "Path and suites are incomptiable options."
    assert (
        exc_info.value.resolution
        == "Verify the repository configuration and remove 'path' or 'suites'."
    )


def test_apt_invalid_path_with_components():
    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryApt(
            key_id="A" * 40,
            path="/",
            components=["main"],
            url="http://archive.ubuntu.com/ubuntu",
        )

    assert (
        exc_info.value.brief == "Components ['main'] cannot be combined with path '/'."
    )
    assert exc_info.value.details == "Path and components are incomptiable options."
    assert (
        exc_info.value.resolution
        == "Verify the repository configuration and remove 'path' or 'components'."
    )


def test_apt_invalid_missing_components():
    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryApt(
            key_id="A" * 40,
            suites=["xenial", "xenial-updates"],
            url="http://archive.ubuntu.com/ubuntu",
        )

    assert exc_info.value.brief == "No components specified."
    assert exc_info.value.details == "Components are required when using suites."
    assert (
        exc_info.value.resolution
        == "Verify the repository configuration and ensure that 'components' is correctly specified."
    )


def test_apt_invalid_missing_suites():
    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryApt(
            key_id="A" * 40,
            components=["main"],
            url="http://archive.ubuntu.com/ubuntu",
        )

    assert exc_info.value.brief == "No suites specified."
    assert exc_info.value.details == "Suites are required when using components."
    assert (
        exc_info.value.resolution
        == "Verify the repository configuration and ensure that 'suites' is correctly specified."
    )


def test_apt_invalid_suites_as_path():
    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryApt(
            key_id="A" * 40,
            suites=["my-suite/"],
            url="http://archive.ubuntu.com/ubuntu",
        )

    assert exc_info.value.brief == "Invalid suite 'my-suite/'."
    assert exc_info.value.details == "Suites must not end with a '/'."
    assert (
        exc_info.value.resolution
        == "Verify the repository configuration and remove the trailing '/ from suites or use the 'path' property to define a path."
    )


def test_apt_marshal():
    repo = PackageRepositoryApt(
        architectures=["amd64", "i386"],
        components=["main", "multiverse"],
        formats=["deb", "deb-src"],
        key_id="A" * 40,
        key_server="xkeyserver.ubuntu.com",
        name="test-name",
        suites=["xenial", "xenial-updates"],
        url="http://archive.ubuntu.com/ubuntu",
    )

    assert repo.marshal() == {
        "architectures": ["amd64", "i386"],
        "components": ["main", "multiverse"],
        "formats": ["deb", "deb-src"],
        "key-id": "A" * 40,
        "key-server": "xkeyserver.ubuntu.com",
        "name": "test-name",
        "suites": ["xenial", "xenial-updates"],
        "type": "apt",
        "url": "http://archive.ubuntu.com/ubuntu",
    }


def test_apt_unmarshal_invalid_extra_keys():
    test_dict = {
        "architectures": ["amd64", "i386"],
        "components": ["main", "multiverse"],
        "formats": ["deb", "deb-src"],
        "key-id": "A" * 40,
        "key-server": "keyserver.ubuntu.com",
        "name": "test-name",
        "suites": ["xenial", "xenial-updates"],
        "type": "apt",
        "url": "http://archive.ubuntu.com/ubuntu",
        "foo": "bar",
        "foo2": "bar",
    }

    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryApt.unmarshal(test_dict)

    assert (
        exc_info.value.brief
        == "Found unsupported package repository properties 'foo', 'foo2'."
    )
    assert exc_info.value.details is None
    assert (
        exc_info.value.resolution
        == "Verify repository configuration and ensure it is correct."
    )


def test_apt_unmarshal_invalid_data():
    test_dict = "not-a-dict"

    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryApt.unmarshal(test_dict)

    assert exc_info.value.brief == "Invalid package repository object 'not-a-dict'."
    assert (
        exc_info.value.details
        == "Package repository must be a valid dictionary object."
    )
    assert (
        exc_info.value.resolution
        == "Verify repository configuration and ensure that the correct syntax is used."
    )


def test_apt_unmarshal_invalid_type():
    test_dict = {
        "architectures": ["amd64", "i386"],
        "components": ["main", "multiverse"],
        "formats": ["deb", "deb-src"],
        "key-id": "A" * 40,
        "key-server": "keyserver.ubuntu.com",
        "name": "test-name",
        "suites": ["xenial", "xenial-updates"],
        "type": "aptx",
        "url": "http://archive.ubuntu.com/ubuntu",
    }

    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryApt.unmarshal(test_dict)

    assert exc_info.value.brief == "Unsupported type 'aptx'."
    assert exc_info.value.details == "The only currently supported type is 'apt'."
    assert (
        exc_info.value.resolution
        == "Verify repository configuration and ensure that 'type' is correctly specified."
    )


def test_ppa_marshal():
    repo = PackageRepositoryAptPpa(ppa="test/ppa")

    assert repo.marshal() == {"type": "apt", "ppa": "test/ppa"}


def test_ppa_invalid_ppa():
    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryAptPpa(ppa="")

    assert exc_info.value.brief == "Invalid PPA ''."
    assert exc_info.value.details == "PPAs must be non-empty strings."
    assert (
        exc_info.value.resolution
        == "Verify repository configuration and ensure that 'ppa' is correctly specified."
    )


def test_ppa_unmarshal_invalid_data():
    test_dict = "not-a-dict"

    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryAptPpa.unmarshal(test_dict)

    assert exc_info.value.brief == "Invalid package repository object 'not-a-dict'."
    assert (
        exc_info.value.details
        == "Package repository must be a valid dictionary object."
    )
    assert (
        exc_info.value.resolution
        == "Verify repository configuration and ensure that the correct syntax is used."
    )


def test_ppa_unmarshal_invalid_apt_ppa_type():
    test_dict = {"type": "aptx", "ppa": "test/ppa"}

    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryAptPpa.unmarshal(test_dict)

    assert exc_info.value.brief == "Unsupported type 'aptx'."
    assert exc_info.value.details == "The only currently supported type is 'apt'."
    assert (
        exc_info.value.resolution
        == "Verify repository configuration and ensure that 'type' is correctly specified."
    )


def test_ppa_unmarshal_invalid_apt_ppa_extra_keys():
    test_dict = {"type": "apt", "ppa": "test/ppa", "test": "foo"}

    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryAptPpa.unmarshal(test_dict)

    assert (
        exc_info.value.brief
        == "Found unsupported package repository properties 'test'."
    )
    assert exc_info.value.details is None
    assert (
        exc_info.value.resolution
        == "Verify repository configuration and ensure that it is correct."
    )


def test_unmarshal_package_repositories_list_none():
    assert PackageRepository.unmarshal_package_repositories(None) == list()


def test_unmarshal_package_repositories_list_empty():
    assert PackageRepository.unmarshal_package_repositories(list()) == list()


def test_unmarshal_package_repositories_list_ppa():
    test_dict = {"type": "apt", "ppa": "test/foo"}
    test_list = [test_dict]

    unmarshalled_list = [
        repo.marshal()
        for repo in PackageRepository.unmarshal_package_repositories(test_list)
    ]

    assert unmarshalled_list == test_list


def test_unmarshal_package_repositories_list_apt():
    test_dict = {
        "architectures": ["amd64", "i386"],
        "components": ["main", "multiverse"],
        "formats": ["deb", "deb-src"],
        "key-id": "A" * 40,
        "key-server": "keyserver.ubuntu.com",
        "name": "test-name",
        "suites": ["xenial", "xenial-updates"],
        "type": "apt",
        "url": "http://archive.ubuntu.com/ubuntu",
    }

    test_list = [test_dict]

    unmarshalled_list = [
        repo.marshal()
        for repo in PackageRepository.unmarshal_package_repositories(test_list)
    ]

    assert unmarshalled_list == test_list


def test_unmarshal_package_repositories_list_all():
    test_ppa = {"type": "apt", "ppa": "test/foo"}

    test_deb = {
        "architectures": ["amd64", "i386"],
        "components": ["main", "multiverse"],
        "formats": ["deb", "deb-src"],
        "key-id": "A" * 40,
        "key-server": "keyserver.ubuntu.com",
        "name": "test-name",
        "suites": ["xenial", "xenial-updates"],
        "type": "apt",
        "url": "http://archive.ubuntu.com/ubuntu",
    }

    test_list = [test_ppa, test_deb]

    unmarshalled_list = [
        repo.marshal()
        for repo in PackageRepository.unmarshal_package_repositories(test_list)
    ]

    assert unmarshalled_list == test_list


def test_unmarshal_package_repositories_invalid_data():
    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepository.unmarshal_package_repositories("not-a-list")

    assert (
        exc_info.value.brief == "Invalid package-repositories list object 'not-a-list'."
    )
    assert exc_info.value.details == "Package repositories must be a list of objects."
    assert (
        exc_info.value.resolution
        == "Verify 'package-repositories' configuration and ensure that the correct syntax is used."
    )
