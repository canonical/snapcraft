# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

import json
from unittest.mock import call

import craft_store
import pytest
import requests
from craft_store import endpoints

from snapcraft.commands.store import client
from snapcraft.utils import OSPlatform

from .utils import FakeResponse

####################
# User Agent Tests #
####################


def test_useragent_linux():
    """Construct a user-agent as a patched Linux machine"""
    os_platform = OSPlatform(
        system="Arch Linux", release="5.10.10-arch1-1", machine="x86_64"
    )

    assert client.build_user_agent(version="7.1.0", os_platform=os_platform) == (
        "snapcraft/7.1.0 Arch Linux/5.10.10-arch1-1 (x86_64)"
    )


@pytest.mark.parametrize("testing_env", ("TRAVIS_TESTING", "AUTOPKGTEST_TMP"))
def test_useragent_linux_with_testing(monkeypatch, testing_env):
    """Construct a user-agent as a patched Linux machine"""
    monkeypatch.setenv(testing_env, "1")
    os_platform = OSPlatform(
        system="Arch Linux", release="5.10.10-arch1-1", machine="x86_64"
    )

    assert client.build_user_agent(version="7.1.0", os_platform=os_platform) == (
        "snapcraft/7.1.0 (testing) Arch Linux/5.10.10-arch1-1 (x86_64)"
    )


@pytest.mark.parametrize("testing_env", ("TRAVIS_TESTING", "AUTOPKGTEST_TMP"))
def test_useragent_windows_with_testing(monkeypatch, testing_env):
    """Construct a user-agent as a patched Windows machine"""
    monkeypatch.setenv(testing_env, "1")
    os_platform = OSPlatform(system="Windows", release="10", machine="AMD64")

    assert client.build_user_agent(version="7.1.0", os_platform=os_platform) == (
        "snapcraft/7.1.0 (testing) Windows/10 (AMD64)"
    )


#####################
# Store Environment #
#####################


@pytest.mark.parametrize("env, expected", (("candid", True), ("not-candid", False)))
def test_use_candid(monkeypatch, env, expected):
    monkeypatch.setenv("SNAPCRAFT_STORE_AUTH", env)

    assert client.use_candid() is expected


def test_get_store_url():
    assert client.get_store_url() == "https://dashboard.snapcraft.io"


def test_get_store_url_from_env(monkeypatch):
    monkeypatch.setenv("STORE_DASHBOARD_URL", "https://fake-store.io")

    assert client.get_store_url() == "https://fake-store.io"


def test_get_store_upload_url():
    assert client.get_store_upload_url() == "https://storage.snapcraftcontent.com"


def test_get_store_url_upload_from_env(monkeypatch):
    monkeypatch.setenv("STORE_UPLOAD_URL", "https://fake-store-upload.io")

    assert client.get_store_upload_url() == "https://fake-store-upload.io"


def test_get_store_login_url():
    assert client.get_store_login_url() == "https://login.ubuntu.com"


def test_get_store_login_from_env(monkeypatch):
    monkeypatch.setenv("UBUNTU_ONE_SSO_URL", "https://fake-login.io")

    assert client.get_store_login_url() == "https://fake-login.io"


####################
# Host Environment #
####################


def test_get_hostname_none_is_unkown():
    assert client._get_hostname(hostname=None) == "UNKNOWN"


def test_get_hostname():
    assert client._get_hostname(hostname="acme") == "acme"


#######################
# StoreClient factory #
#######################


@pytest.mark.parametrize("ephemeral", (True, False))
def test_get_store_client(monkeypatch, ephemeral):
    monkeypatch.setenv("SNAPCRAFT_STORE_AUTH", "candid")

    store_client = client.get_client(ephemeral)

    assert isinstance(store_client, craft_store.StoreClient)


@pytest.mark.parametrize("ephemeral", (True, False))
def test_get_ubuntu_client(ephemeral):
    store_client = client.get_client(ephemeral)

    assert isinstance(store_client, craft_store.UbuntuOneStoreClient)


##################
# StoreClientCLI #
##################


@pytest.fixture
def fake_user_password(mocker):
    """Return a canned user name and password"""
    mocker.patch.object(
        client,
        "_prompt_login",
        return_value=("fake-username@acme.com", "fake-password"),
    )


@pytest.fixture
def fake_otp(mocker):
    """Return a canned user name and password"""
    mocker.patch.object(
        client.utils,
        "prompt",
        return_value="123456",
    )


@pytest.fixture
def fake_hostname(mocker):
    mocker.patch.object(client, "_get_hostname", return_value="fake-host")


@pytest.mark.usefixtures("fake_user_password", "fake_hostname")
def test_login(fake_client):
    client.StoreClientCLI().login()

    assert fake_client.login.mock_calls == [
        call(
            ttl=31536000,
            permissions=[
                "package_access",
                "package_manage",
                "package_metrics",
                "package_push",
                "package_register",
                "package_release",
                "package_update",
            ],
            channels=None,
            packages=[],
            description="snapcraft@fake-host",
            email="fake-username@acme.com",
            password="fake-password",
        )
    ]


@pytest.mark.usefixtures("fake_user_password", "fake_otp", "fake_hostname")
def test_login_otp(fake_client):
    fake_client.login.side_effect = [
        craft_store.errors.StoreServerError(
            FakeResponse(
                status_code=requests.codes.unauthorized,  # pylint: disable=no-member
                content=json.dumps(
                    {"error_list": [{"message": "2fa", "code": "twofactor-required"}]}
                ),
            )
        ),
        None,
    ]

    client.StoreClientCLI().login()

    assert fake_client.login.mock_calls == [
        call(
            ttl=31536000,
            permissions=[
                "package_access",
                "package_manage",
                "package_metrics",
                "package_push",
                "package_register",
                "package_release",
                "package_update",
            ],
            channels=None,
            packages=[],
            description="snapcraft@fake-host",
            email="fake-username@acme.com",
            password="fake-password",
        ),
        call(
            ttl=31536000,
            permissions=[
                "package_access",
                "package_manage",
                "package_metrics",
                "package_push",
                "package_register",
                "package_release",
                "package_update",
            ],
            channels=None,
            packages=[],
            description="snapcraft@fake-host",
            email="fake-username@acme.com",
            password="fake-password",
            otp="123456",
        ),
    ]


@pytest.mark.usefixtures("fake_user_password", "fake_hostname")
def test_with_params(fake_client):
    client.StoreClientCLI().login(
        ttl=20,
        acls=["package_access", "package_push"],
        packages=["fake-snap", "fake-other-snap"],
        channels=["stable/fake", "edge/fake"],
    )

    assert fake_client.login.mock_calls == [
        call(
            ttl=20,
            permissions=[
                "package_access",
                "package_push",
            ],
            channels=["stable/fake", "edge/fake"],
            packages=[
                endpoints.Package(package_name="fake-snap", package_type="snap"),
                endpoints.Package(package_name="fake-other-snap", package_type="snap"),
            ],
            description="snapcraft@fake-host",
            email="fake-username@acme.com",
            password="fake-password",
        )
    ]
