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


import subprocess
from unittest import mock
from unittest.mock import call

import gnupg
import pytest

from snapcraft.internal.meta.package_repository import (
    PackageRepositoryApt,
    PackageRepositoryAptPpa,
)
from snapcraft.internal.repo import apt_ppa, errors
from snapcraft.internal.repo.apt_key_manager import AptKeyManager


@pytest.fixture(autouse=True)
def mock_environ_copy():
    with mock.patch("os.environ.copy") as m:
        yield m


@pytest.fixture(autouse=True)
def mock_gnupg(tmp_path, autouse=True):
    with mock.patch("gnupg.GPG", spec=gnupg.GPG) as m:
        m.return_value.import_keys.return_value.fingerprints = [
            "FAKE-KEY-ID-FROM-GNUPG"
        ]
        yield m


@pytest.fixture(autouse=True)
def mock_run():
    with mock.patch("subprocess.run", spec=subprocess.run) as m:
        yield m


@pytest.fixture(autouse=True)
def mock_apt_ppa_get_signing_key():
    with mock.patch(
        "snapcraft.internal.repo.apt_ppa.get_launchpad_ppa_key_id",
        spec=apt_ppa.get_launchpad_ppa_key_id,
        return_value="FAKE-PPA-SIGNING-KEY",
    ) as m:
        yield m


@pytest.fixture
def key_assets(tmp_path):
    key_assets = tmp_path / "key-assets"
    key_assets.mkdir(parents=True)
    yield key_assets


@pytest.fixture
def gpg_keyring(tmp_path):
    yield tmp_path / "keyring.gpg"


@pytest.fixture
def apt_gpg(key_assets, gpg_keyring):
    yield AptKeyManager(
        gpg_keyring=gpg_keyring, key_assets=key_assets,
    )


def test_find_asset(
    apt_gpg, key_assets,
):
    key_id = "8" * 40
    expected_key_path = key_assets / ("8" * 8 + ".asc")
    expected_key_path.write_text("key")

    key_path = apt_gpg.find_asset_with_key_id(key_id=key_id)

    assert key_path == expected_key_path


def test_find_asset_none(apt_gpg,):
    key_path = apt_gpg.find_asset_with_key_id(key_id="foo")

    assert key_path is None


def test_get_key_fingerprints(
    apt_gpg, mock_gnupg,
):
    with mock.patch("tempfile.NamedTemporaryFile") as m:
        m.return_value.__enter__.return_value.name = "/tmp/foo"
        ids = apt_gpg.get_key_fingerprints(key="8" * 40)

    assert ids == ["FAKE-KEY-ID-FROM-GNUPG"]
    assert mock_gnupg.mock_calls == [
        call(keyring="/tmp/foo"),
        call().import_keys(key_data="8888888888888888888888888888888888888888"),
    ]


@pytest.mark.parametrize(
    "stdout,expected",
    [
        (b"nothing exported", False),
        (b"BEGIN PGP PUBLIC KEY BLOCK", True),
        (b"invalid", False),
    ],
)
def test_is_key_installed(
    stdout, expected, apt_gpg, mock_run,
):
    mock_run.return_value.stdout = stdout

    is_installed = apt_gpg.is_key_installed(key_id="foo")

    assert is_installed is expected
    assert mock_run.mock_calls == [
        call(
            ["sudo", "apt-key", "export", "foo"],
            check=True,
            stderr=subprocess.STDOUT,
            stdout=subprocess.PIPE,
        )
    ]


def test_is_key_installed_with_apt_key_failure(
    apt_gpg, mock_run,
):
    mock_run.side_effect = subprocess.CalledProcessError(
        cmd=["apt-key"], returncode=1, output=b"some error"
    )

    is_installed = apt_gpg.is_key_installed(key_id="foo")

    assert is_installed is False


def test_install_key(
    apt_gpg, gpg_keyring, mock_run,
):
    key = "some-fake-key"
    apt_gpg.install_key(key=key)

    assert mock_run.mock_calls == [
        call(
            ["sudo", "apt-key", "--keyring", str(gpg_keyring), "add", "-"],
            check=True,
            env={"LANG": "C.UTF-8"},
            input=b"some-fake-key",
            stderr=subprocess.STDOUT,
            stdout=subprocess.PIPE,
        )
    ]


def test_install_key_with_apt_key_failure(apt_gpg, mock_run):
    mock_run.side_effect = subprocess.CalledProcessError(
        cmd=["foo"], returncode=1, output=b"some error"
    )

    with pytest.raises(errors.AptGPGKeyInstallError) as exc_info:
        apt_gpg.install_key(key="FAKEKEY")

    assert exc_info.value._output == "some error"
    assert exc_info.value._key == "FAKEKEY"


def test_install_key_from_keyserver(apt_gpg, gpg_keyring, mock_run):
    apt_gpg.install_key_from_keyserver(key_id="FAKE_KEYID", key_server="key.server")

    assert mock_run.mock_calls == [
        call(
            [
                "sudo",
                "apt-key",
                "--keyring",
                str(gpg_keyring),
                "adv",
                "--keyserver",
                "key.server",
                "--recv-keys",
                "FAKE_KEYID",
            ],
            check=True,
            env={"LANG": "C.UTF-8"},
            stderr=subprocess.STDOUT,
            stdout=subprocess.PIPE,
        )
    ]


def test_install_key_from_keyserver_with_apt_key_failure(
    apt_gpg, gpg_keyring, mock_run
):
    mock_run.side_effect = subprocess.CalledProcessError(
        cmd=["apt-key"], returncode=1, output=b"some error"
    )

    with pytest.raises(errors.AptGPGKeyInstallError) as exc_info:
        apt_gpg.install_key_from_keyserver(
            key_id="fake-key-id", key_server="fake-server"
        )

    assert exc_info.value._output == "some error"
    assert exc_info.value._key_id == "fake-key-id"


@mock.patch("snapcraft.internal.repo.apt_key_manager.AptKeyManager.is_key_installed")
@pytest.mark.parametrize(
    "is_installed", [True, False],
)
def test_install_package_repository_key_already_installed(
    mock_is_key_installed, is_installed, apt_gpg,
):
    mock_is_key_installed.return_value = is_installed
    package_repo = PackageRepositoryApt(
        components=["main", "multiverse"],
        key_id="8" * 40,
        key_server="xkeyserver.com",
        suites=["xenial"],
        url="http://archive.ubuntu.com/ubuntu",
    )

    updated = apt_gpg.install_package_repository_key(package_repo=package_repo)

    assert updated is not is_installed


@mock.patch(
    "snapcraft.internal.repo.apt_key_manager.AptKeyManager.is_key_installed",
    return_value=False,
)
@mock.patch("snapcraft.internal.repo.apt_key_manager.AptKeyManager.install_key")
def test_install_package_repository_key_from_asset(
    mock_install_key, mock_is_key_installed, apt_gpg, key_assets,
):
    key_id = "123456789012345678901234567890123456AABB"
    expected_key_path = key_assets / "3456AABB.asc"
    expected_key_path.write_text("key-data")

    package_repo = PackageRepositoryApt(
        components=["main", "multiverse"],
        key_id=key_id,
        suites=["xenial"],
        url="http://archive.ubuntu.com/ubuntu",
    )

    updated = apt_gpg.install_package_repository_key(package_repo=package_repo)

    assert updated is True
    assert mock_install_key.mock_calls == [call(key="key-data")]


@mock.patch(
    "snapcraft.internal.repo.apt_key_manager.AptKeyManager.is_key_installed",
    return_value=False,
)
@mock.patch(
    "snapcraft.internal.repo.apt_key_manager.AptKeyManager.install_key_from_keyserver"
)
def test_install_package_repository_key_apt_from_keyserver(
    mock_install_key_from_keyserver, mock_is_key_installed, apt_gpg,
):
    key_id = "8" * 40

    package_repo = PackageRepositoryApt(
        components=["main", "multiverse"],
        key_id=key_id,
        key_server="key.server",
        suites=["xenial"],
        url="http://archive.ubuntu.com/ubuntu",
    )

    updated = apt_gpg.install_package_repository_key(package_repo=package_repo)

    assert updated is True
    assert mock_install_key_from_keyserver.mock_calls == [
        call(key_id=key_id, key_server="key.server")
    ]


@mock.patch(
    "snapcraft.internal.repo.apt_key_manager.AptKeyManager.is_key_installed",
    return_value=False,
)
@mock.patch(
    "snapcraft.internal.repo.apt_key_manager.AptKeyManager.install_key_from_keyserver"
)
def test_install_package_repository_key_ppa_from_keyserver(
    mock_install_key_from_keyserver, mock_is_key_installed, apt_gpg,
):
    package_repo = PackageRepositoryAptPpa(ppa="test/ppa",)

    updated = apt_gpg.install_package_repository_key(package_repo=package_repo)

    assert updated is True
    assert mock_install_key_from_keyserver.mock_calls == [
        call(key_id="FAKE-PPA-SIGNING-KEY", key_server="keyserver.ubuntu.com")
    ]
