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
import base64
from textwrap import dedent

import craft_store.endpoints
import pymacaroons
import pytest

from snapcraft import errors
from snapcraft.commands.store._legacy_account import LegacyUbuntuOne, get_auth

############
# Fixtures #
############


@pytest.fixture
def fake_auth(monkeypatch):
    monkeypatch.setattr(
        "snapcraft.commands.store._legacy_account.get_auth",
        lambda config_content: base64.b64encode(b"Macaroon root=secret").decode(),
    )


#############
# Macaroons #
#############


def test_invalid_macaroon_root_raises_exception(new_dir):
    config_content = dedent(
        """\
        [login.ubuntu.com]
        macaroon=inval'id
        unbound_discharge=ssssssssssssssssssssssss
        """
    )

    with pytest.raises(errors.SnapcraftError):
        get_auth(config_content)


def test_invalid_discharge_raises_exception():
    config_content = dedent(
        f"""\
        [login.ubuntu.com]
        macaroon={pymacaroons.Macaroon().serialize()}
        unbound_discharge=inval'id
        """
    )

    with pytest.raises(errors.SnapcraftError):
        get_auth(config_content)


########################
# LegacyStoreClientCLI #
########################


def test_store_credentials(legacy_config_path, fake_auth):
    LegacyUbuntuOne.store_credentials("secret")

    assert legacy_config_path.read_text() == "TWFjYXJvb24gcm9vdD1zZWNyZXQ="


@pytest.mark.usefixtures("fake_auth")
def test_logout(legacy_config_path):
    LegacyUbuntuOne.store_credentials("secret")
    assert LegacyUbuntuOne.has_legacy_credentials() is True

    client = LegacyUbuntuOne(
        base_url="",
        storage_base_url="",
        auth_url="",
        endpoints=craft_store.endpoints.U1_SNAP_STORE,
        application_name="snapcraft",
        user_agent="",
    )
    client.logout()

    assert legacy_config_path.exists() is False


def test_logout_file_missing(legacy_config_path):
    u1_client = LegacyUbuntuOne(
        base_url="",
        storage_base_url="",
        auth_url="",
        endpoints=craft_store.endpoints.U1_SNAP_STORE,
        application_name="snapcraft",
        user_agent="",
    )
    u1_client.logout()

    assert legacy_config_path.exists() is False


def test_login(legacy_config_path, fake_auth):
    client = LegacyUbuntuOne(
        base_url="",
        storage_base_url="",
        auth_url="",
        endpoints=craft_store.endpoints.U1_SNAP_STORE,
        application_name="snapcraft",
        user_agent="",
    )
    with pytest.raises(NotImplementedError):
        client.login(
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


def test_request(mocker, legacy_config_path, fake_auth):
    request_mock = mocker.patch(
        "craft_store.base_client.HTTPClient.request",
        autospec=True,
    )
    LegacyUbuntuOne.store_credentials("secret")
    assert LegacyUbuntuOne.has_legacy_credentials() is True

    client = LegacyUbuntuOne(
        base_url="",
        storage_base_url="",
        auth_url="",
        endpoints=craft_store.endpoints.U1_SNAP_STORE,
        application_name="snapcraft",
        user_agent="",
    )
    client.request("GET", "https://foo.com")
    request_mock.assert_called_once_with(
        client.http_client,
        "GET",
        "https://foo.com",
        headers={"Authorization": "Macaroon root=secret"},
        params=None,
    )
