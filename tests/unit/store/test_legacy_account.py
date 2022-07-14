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
import json
from textwrap import dedent

import craft_store.endpoints
import pytest

from snapcraft import errors
from snapcraft.store._legacy_account import LegacyUbuntuOne, get_auth

############
# Fixtures #
############


@pytest.fixture
def fake_auth(root_macaroon, discharged_macaroon):
    return base64.b64encode(
        json.dumps({"r": root_macaroon, "d": discharged_macaroon}).encode()
    ).decode()


@pytest.fixture
def fake_get_auth(monkeypatch, fake_auth):
    monkeypatch.setattr(
        "snapcraft.store._legacy_account.get_auth",
        lambda config_content: fake_auth,
    )


#############
# Macaroons #
#############


def test_get_auth(new_dir, root_macaroon, discharged_macaroon):
    config_content = dedent(
        f"""\
        [login.ubuntu.com]
        macaroon={root_macaroon}
        unbound_discharge={discharged_macaroon}
        """
    )

    auth = json.loads(base64.b64decode(get_auth(config_content).encode()))

    assert auth["r"] == root_macaroon
    assert auth["d"] == discharged_macaroon


def test_get_auth_missing_macaroon(new_dir, discharged_macaroon):
    config_content = dedent(
        f"""\
        [login.ubuntu.com]
        unbound_discharge={discharged_macaroon}
        """
    )

    with pytest.raises(errors.SnapcraftError):
        get_auth(config_content)


def test_get_auth_missing_discharge(new_dir, root_macaroon):
    config_content = dedent(
        f"""\
        [login.ubuntu.com]
        root={root_macaroon}
        """
    )

    with pytest.raises(errors.SnapcraftError):
        get_auth(config_content)


########################
# LegacyStoreClientCLI #
########################


def test_store_credentials(legacy_config_path):
    LegacyUbuntuOne.store_credentials("secret-config")

    assert legacy_config_path.read_text() == "secret-config"


def test_legacy_credentials_in_env(monkeypatch, legacy_config_credentials):
    monkeypatch.setenv("SNAPCRAFT_STORE_CREDENTIALS", legacy_config_credentials)

    assert LegacyUbuntuOne.env_has_legacy_credentials() is True


def test_legacy_credentials_in_env_false_when_no_env(monkeypatch):
    monkeypatch.delenv("SNAPCRAFT_STORE_CREDENTIALS", raising=False)

    assert LegacyUbuntuOne.env_has_legacy_credentials() is False


def test_logout(legacy_config_path):
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


def test_login():
    client = LegacyUbuntuOne(
        base_url="",
        storage_base_url="",
        auth_url="",
        endpoints=craft_store.endpoints.U1_SNAP_STORE,
        application_name="snapcraft",
        user_agent="",
    )
    with pytest.raises(errors.SnapcraftError):
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


def test_request(mocker, legacy_config_path, fake_get_auth):
    request_mock = mocker.patch(
        "craft_store.base_client.HTTPClient.request",
        autospec=True,
    )
    assert LegacyUbuntuOne.has_legacy_credentials() is True
    assert LegacyUbuntuOne.env_has_legacy_credentials() is False

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
        headers={
            "Authorization": (
                "Macaroon "
                "root=MDAxZGxvY2F0aW9uIGZha2Utc2VydmVyLmNvbQowMDEwaWRlbnRpZml"
                "lciAKMDAxM2NpZCAxMjM0NTY3ODkwCjAwMTN2aWQgMTIzNDU2Nzg5MAowMDE"
                "0Y2wgZmFrZS1zc28uY29tCjAwMmZzaWduYXR1cmUg2VM0YdeDXkhRx-O2ORR"
                "EBs92hZfepuEzIy-9I4WlwFAK, "
                "discharge=MDAxZGxvY2F0aW9uIGZha2Utc2VydmVyLmNvbQowMDEwaWRlbn"
                "RpZmllciAKMDAyZnNpZ25hdHVyZSB6hf06Su8kgum0keaUXy6VxGUHlN9bFL"
                "2A0EKNptFZMwo"
            )
        },
        params=None,
    )


def test_request_with_env(
    monkeypatch, mocker, legacy_config_path, legacy_config_credentials, fake_get_auth
):
    request_mock = mocker.patch(
        "craft_store.base_client.HTTPClient.request",
        autospec=True,
    )
    legacy_config_path.unlink()
    assert LegacyUbuntuOne.has_legacy_credentials() is False
    monkeypatch.setenv("SNAPCRAFT_STORE_CREDENTIALS", legacy_config_credentials)
    assert LegacyUbuntuOne.env_has_legacy_credentials() is True

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
        headers={
            "Authorization": (
                "Macaroon "
                "root=MDAxZGxvY2F0aW9uIGZha2Utc2VydmVyLmNvbQowMDEwaWRlbnRpZml"
                "lciAKMDAxM2NpZCAxMjM0NTY3ODkwCjAwMTN2aWQgMTIzNDU2Nzg5MAowMDE"
                "0Y2wgZmFrZS1zc28uY29tCjAwMmZzaWduYXR1cmUg2VM0YdeDXkhRx-O2ORR"
                "EBs92hZfepuEzIy-9I4WlwFAK, "
                "discharge=MDAxZGxvY2F0aW9uIGZha2Utc2VydmVyLmNvbQowMDEwaWRlbn"
                "RpZmllciAKMDAyZnNpZ25hdHVyZSB6hf06Su8kgum0keaUXy6VxGUHlN9bFL"
                "2A0EKNptFZMwo"
            )
        },
        params=None,
    )
