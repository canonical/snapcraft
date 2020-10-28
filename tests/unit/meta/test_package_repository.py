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

from snapcraft.internal import repo
from snapcraft.internal.meta import errors
from snapcraft.internal.meta.package_repository import (
    PackageRepository,
    PackageRepositoryApt,
    PackageRepositoryAptPpa,
)
from tests.unit import mock


@pytest.fixture(autouse=True)
def mock_repo():
    with mock.patch(
        "snapcraft.internal.meta.package_repository.Ubuntu", spec=repo.Ubuntu
    ) as m:
        yield m


def test_apt_name():
    repo = PackageRepositoryApt(
        architectures=["amd64", "i386"],
        components=["main", "multiverse"],
        formats=["deb", "deb-src"],
        key_id="test-key-id",
        key_server="keyserver.ubuntu.com",
        suites=["xenial", "xenial-updates"],
        url="http://archive.ubuntu.com/ubuntu",
    )

    assert repo.name == "http_archive_ubuntu_com_ubuntu"


def test_apt_invalid_url():
    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryApt(
            key_id="test-key-id", url="",
        )

    assert exc_info.value.brief == "Invalid URL ''."
    assert exc_info.value.resolution == "You can update 'url' to a non-empty string."


def test_apt_invalid_path():
    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryApt(
            key_id="test-key-id", path="", url="http://archive.ubuntu.com/ubuntu",
        )

    assert exc_info.value.brief == "Invalid path ''."
    assert (
        exc_info.value.resolution
        == "You can update 'path' to a non-empty string, such as '/'."
    )


def test_apt_invalid_path_with_suites():
    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryApt(
            key_id="test-key-id",
            path="/",
            suites=["xenial", "xenial-updates"],
            url="http://archive.ubuntu.com/ubuntu",
        )

    assert exc_info.value.brief == "Components and suites cannot be used with path."
    assert (
        exc_info.value.resolution
        == "Paths and components/suites are mutually exclusive options.  You can remove the path, or components and suites."
    )


def test_apt_invalid_path_with_components():
    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryApt(
            key_id="test-key-id",
            path="/",
            components=["main"],
            url="http://archive.ubuntu.com/ubuntu",
        )

    assert exc_info.value.brief == "Components and suites cannot be used with path."
    assert (
        exc_info.value.resolution
        == "Paths and components/suites are mutually exclusive options.  You can remove the path, or components and suites."
    )


def test_apt_invalid_missing_components():
    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryApt(
            key_id="test-key-id",
            suites=["xenial", "xenial-updates"],
            url="http://archive.ubuntu.com/ubuntu",
        )

    assert exc_info.value.brief == "No components specified."
    assert (
        exc_info.value.resolution
        == "Components are required when using suites.  You can correct this by adding the correct components to the repository configuration."
    )


def test_apt_invalid_missing_suites():
    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryApt(
            key_id="test-key-id",
            components=["main"],
            url="http://archive.ubuntu.com/ubuntu",
        )

    assert exc_info.value.brief == "No suites specified."
    assert (
        exc_info.value.resolution
        == "Suites are required when using components.  You can correct this by adding the correct suites to the repository configuration."
    )


def test_apt_invalid_suites_as_path():
    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryApt(
            key_id="test-key-id",
            suites=["my-suite/"],
            url="http://archive.ubuntu.com/ubuntu",
        )

    assert exc_info.value.brief == "Suites cannot end with a '/'."
    assert (
        exc_info.value.resolution
        == "You can either remove the '/' or instead use the 'path' property to define an exact path."
    )


def test_apt_install(mock_repo, tmp_path):
    mock_repo.install_gpg_key_id.return_value = True

    repo = PackageRepositoryApt(
        architectures=["amd64", "i386"],
        components=["main", "multiverse"],
        formats=["deb", "deb-src"],
        key_id="test-key-id",
        key_server="xkeyserver.ubuntu.com",
        name="some-name",
        suites=["xenial", "xenial-updates"],
        url="http://archive.ubuntu.com/ubuntu",
    )

    repo.install(keys_path=tmp_path)

    assert mock_repo.mock_calls == [
        mock.call.install_gpg_key_id(
            key_id="test-key-id",
            key_server="xkeyserver.ubuntu.com",
            keys_path=tmp_path,
        ),
        mock.call.install_sources(
            architectures=["amd64", "i386"],
            components=["main", "multiverse"],
            formats=["deb", "deb-src"],
            name="some-name",
            suites=["xenial", "xenial-updates"],
            url="http://archive.ubuntu.com/ubuntu",
        ),
    ]


def test_apt_install_with_path(mock_repo, tmp_path):
    mock_repo.install_gpg_key_id.return_value = True

    repo = PackageRepositoryApt(
        key_id="test-key-id", path="x", url="http://archive.ubuntu.com/ubuntu",
    )

    repo.install(keys_path=tmp_path)

    assert mock_repo.mock_calls == [
        mock.call.install_gpg_key_id(
            key_id="test-key-id", key_server=None, keys_path=tmp_path,
        ),
        mock.call.install_sources(
            architectures=None,
            components=None,
            formats=None,
            name="http_archive_ubuntu_com_ubuntu",
            suites=["x/"],
            url="http://archive.ubuntu.com/ubuntu",
        ),
    ]


def test_apt_install_implied_path(mock_repo, tmp_path):
    mock_repo.install_gpg_key_id.return_value = True

    repo = PackageRepositoryApt(
        key_id="test-key-id", url="http://archive.ubuntu.com/ubuntu",
    )

    repo.install(keys_path=tmp_path)

    assert mock_repo.mock_calls == [
        mock.call.install_gpg_key_id(
            key_id="test-key-id", key_server=None, keys_path=tmp_path,
        ),
        mock.call.install_sources(
            architectures=None,
            components=None,
            formats=None,
            name="http_archive_ubuntu_com_ubuntu",
            suites=["/"],
            url="http://archive.ubuntu.com/ubuntu",
        ),
    ]


def test_apt_marshal():
    repo = PackageRepositoryApt(
        architectures=["amd64", "i386"],
        components=["main", "multiverse"],
        formats=["deb", "deb-src"],
        key_id="test-key-id",
        key_server="xkeyserver.ubuntu.com",
        name="test-name",
        suites=["xenial", "xenial-updates"],
        url="http://archive.ubuntu.com/ubuntu",
    )

    assert repo.marshal() == {
        "architectures": ["amd64", "i386"],
        "components": ["main", "multiverse"],
        "formats": ["deb", "deb-src"],
        "key-id": "test-key-id",
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
        "key-id": "test-key-id",
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

    assert exc_info.value.brief == "Invalid key(s) present ('foo', 'foo2')."


def test_apt_unmarshal_invalid_type():
    test_dict = {
        "architectures": ["amd64", "i386"],
        "components": ["main", "multiverse"],
        "formats": ["deb", "deb-src"],
        "key-id": "test-key-id",
        "key-server": "keyserver.ubuntu.com",
        "name": "test-name",
        "suites": ["xenial", "xenial-updates"],
        "type": "aptx",
        "url": "http://archive.ubuntu.com/ubuntu",
    }

    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryApt.unmarshal(test_dict)

    assert exc_info.value.brief == "Unsupported type 'aptx'."
    assert exc_info.value.resolution == "You can use type 'apt'."


def test_ppa_marshal():
    repo = PackageRepositoryAptPpa(ppa="test/ppa")

    assert repo.marshal() == {"type": "apt", "ppa": "test/ppa"}


def test_ppa_invalid_ppa():
    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryAptPpa(ppa="")

    assert exc_info.value.brief == "Invalid PPA ''."
    assert exc_info.value.resolution == "You can update 'ppa' to a non-empty string."


def test_ppa_unmarshal_invalid_apt_ppa_type():
    test_dict = {"type": "aptx", "ppa": "test/ppa"}

    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryAptPpa.unmarshal(test_dict)

    assert exc_info.value.brief == "Unsupported type 'aptx'."
    assert exc_info.value.resolution == "You can use type 'apt'."


def test_ppa_unmarshal_invalid_apt_ppa_extra_keys():
    test_dict = {"type": "apt", "ppa": "test/ppa", "test": "foo"}

    with pytest.raises(errors.PackageRepositoryValidationError) as exc_info:
        PackageRepositoryAptPpa.unmarshal(test_dict)

    assert exc_info.value.brief == "Invalid keys present ('test')."


def test_ppa_install(tmp_path, mock_repo):
    repo = PackageRepositoryAptPpa(ppa="test/ppa")

    repo.install(keys_path=tmp_path)

    assert mock_repo.mock_calls == [
        mock.call.install_ppa(keys_path=tmp_path, ppa="test/ppa")
    ]


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
        "key-id": "test-key-id",
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
        "key-id": "test-key-id",
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
