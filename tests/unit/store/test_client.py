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
import textwrap
import time
from unittest.mock import ANY, Mock, call

import craft_store
import pytest
import requests
from craft_store import endpoints
from craft_store.models import RevisionsResponseModel

from snapcraft import errors, models
from snapcraft.store import LegacyUbuntuOne, client, constants
from snapcraft.store.channel_map import ChannelMap
from snapcraft_legacy.storeapi.v2.releases import Releases

from .utils import FakeResponse

#############
# Fixtures #
#############


@pytest.fixture
def no_wait(monkeypatch):
    monkeypatch.setattr(time, "sleep", lambda x: None)


@pytest.fixture
def channel_map_payload():
    return {
        "channel-map": [
            {
                "architecture": "all",
                "channel": "2.1/beta",
                "expiration-date": None,
                "revision": 1,
                "progressive": {
                    "paused": None,
                    "percentage": None,
                    "current-percentage": None,
                },
                "when": "2020-02-03T20:58:37Z",
            }
        ],
        "revisions": [
            {
                "architectures": [
                    "amd64",
                    "arm64",
                    "armhf",
                    "i386",
                    "s390x",
                    "ppc64el",
                ],
                "revision": 1,
                "version": "10",
            }
        ],
        "snap": {
            "name": "test-snap",
            "channels": [
                {
                    "branch": None,
                    "fallback": None,
                    "name": "2.1/stable",
                    "risk": "stable",
                    "track": "2.1",
                },
                {
                    "branch": None,
                    "fallback": "2.1/stable",
                    "name": "2.1/candidate",
                    "risk": "candidate",
                    "track": "2.1",
                },
                {
                    "branch": None,
                    "fallback": "2.1/candidate",
                    "name": "2.1/beta",
                    "risk": "beta",
                    "track": "2.1",
                },
                {
                    "branch": None,
                    "fallback": "2.1/beta",
                    "name": "2.1/edge",
                    "risk": "edge",
                    "track": "2.1",
                },
            ],
            "tracks": [
                {
                    "name": "latest",
                    "status": "active",
                    "creation-date": None,
                    "version-pattern": None,
                },
                {
                    "name": "1.0",
                    "status": "default",
                    "creation-date": "2019-10-17T14:11:59Z",
                    "version-pattern": "1.*",
                },
            ],
            "default-track": "2.1",
        },
    }


@pytest.fixture
def list_revisions_payload():
    return {
        "revisions": [
            {
                "architectures": ["i386"],
                "base": "core20",
                "build-url": None,
                "confinement": "strict",
                "created-at": " 2016-09-27T19:23:40Z",
                "grade": "stable",
                "revision": 2,
                "sha3-384": "fake-a9060ef4872ccacbfa44",
                "size": 20,
                "status": "Published",
                "version": "2.0.1",
            },
            {
                "architectures": ["amd64"],
                "base": "core20",
                "build-url": None,
                "confinement": "strict",
                "created-at": "2016-09-27T18:38:43Z",
                "grade": "stable",
                "revision": 1,
                "sha3-384": "fake-a9060ef4872ccacbfa44",
                "size": 20,
                "status": "Published",
                "version": "2.0.2",
            },
        ],
        "releases": [
            {
                "architecture": "amd64",
                "branch": None,
                "channel": "latest/stable",
                "expiration-date": None,
                "revision": 1,
                "risk": "stable",
                "track": "latest",
                "when": "2020-02-12T17:51:40.891996Z",
            },
        ],
    }


@pytest.fixture
def list_validation_sets_payload():
    return {
        "assertions": [
            {
                "headers": {
                    "account_id": "test-account-id",
                    "name": "test-validation-set",
                    "revision": "3",
                    "sequence": "5",
                    "snaps": [
                        {
                            "name": "hello-world",
                            "id": "test-snap-id",
                            "presence": "optional",
                            "revision": "6",
                            "components": {
                                "component-with-revision": {
                                    "presence": "required",
                                    "revision": "10",
                                },
                                "component-without-revision": "invalid",
                            },
                        }
                    ],
                    "authority_id": "test-authority-id",
                    "series": "16",
                    "timestamp": "2026-01-01T10:20:30Z",
                    "type": "validation-set",
                }
            }
        ]
    }


@pytest.fixture
def build_validation_set_payload():
    return {
        "account_id": "test-account-id",
        "name": "test-validation-set",
        "revision": "4",
        "sequence": "5",
        "snaps": [
            {
                "name": "hello-world",
                "id": "test-snap-id",
                "presence": "required",
                "revision": "6",
                "components": {
                    "component-with-revision": {
                        "presence": "required",
                        "revision": "10",
                    },
                    "component-without-revision": "invalid",
                },
            }
        ],
        "authority_id": "test-authority-id",
        "series": "16",
        "timestamp": "2026-01-01T10:20:30Z",
        "type": "validation-set",
    }


@pytest.fixture
def post_validation_set_payload():
    return {
        "assertions": [
            {
                "headers": {
                    "account_id": "test-account-id",
                    "name": "test-validation-set",
                    "revision": "4",
                    "sequence": "5",
                    "snaps": [
                        {
                            "name": "hello-world",
                            "id": "test-snap-id",
                            "presence": "required",
                            "revision": "6",
                            "components": {
                                "component-with-revision": {
                                    "presence": "required",
                                    "revision": "10",
                                },
                                "component-without-revision": "invalid",
                            },
                        }
                    ],
                    "authority_id": "test-authority-id",
                    "series": "16",
                    "timestamp": "2026-01-01T10:20:30Z",
                    "type": "validation-set",
                }
            }
        ]
    }


@pytest.fixture
def list_confdb_schemas_payload():
    return {
        "assertions": [
            {
                "headers": {
                    "account-id": "test-account-id",
                    "authority-id": "test-authority-id",
                    "body-length": "92",
                    "name": "test-confdbs",
                    "revision": "9",
                    "sign-key-sha3-384": "test-sign-key",
                    "timestamp": "2024-01-01T10:20:30Z",
                    "type": "confdb-schema",
                    "views": {
                        "wifi-setup": {
                            "rules": [
                                {
                                    "access": "read-write",
                                    "request": "ssids",
                                    "storage": "wifi.ssids",
                                }
                            ]
                        }
                    },
                },
                "body": '{\n  "storage": {\n    "schema": {\n      "wifi": {\n        "values": "any"\n      }\n    }\n  }\n}',
            },
        ],
    }


@pytest.fixture
def build_confdb_schema_payload():
    return {
        "account_id": "test-account-id",
        "authority_id": "test-authority-id",
        "name": "test-confdbs",
        "revision": "10",
        "views": {
            "wifi-setup": {
                "rules": [
                    {
                        "request": "ssids",
                        "storage": "wifi.ssids",
                        "access": "read-write",
                    }
                ]
            }
        },
        "body": '{\n  "storage": {\n    "schema": {\n      "wifi": {\n        "values": "any"\n      }\n    }\n  }\n}',
        "type": "confdb-schema",
        "timestamp": "2024-01-01T10:20:30Z",
    }


@pytest.fixture
def post_confdb_schema_payload():
    return {
        "assertions": [
            {
                "headers": {
                    "account-id": "test-account-id",
                    "authority-id": "test-authority-id",
                    "body-length": "92",
                    "name": "test-confdbs",
                    "revision": "10",
                    "sign-key-sha3-384": "test-key",
                    "timestamp": "2024-01-01T10:20:30Z",
                    "type": "confdb-schema",
                    "views": {
                        "wifi-setup": {
                            "rules": [
                                {
                                    "access": "read",
                                    "request": "ssids",
                                    "storage": "wifi.ssids",
                                }
                            ]
                        }
                    },
                },
                "body": '{\n  "storage": {\n    "schema": {\n      "wifi": {\n        "values": "any"\n      }\n    }\n  }\n}',
            }
        ]
    }


####################
# User Agent Tests #
####################


def test_useragent_linux(mocker):
    """Construct a user-agent as a patched Linux machine"""
    mocker.patch("distro.id", return_value="Arch Linux")
    mocker.patch("distro.version", return_value="5.10.10-arch1-1")
    mocker.patch(
        "craft_platforms.DebianArchitecture.to_platform_arch", return_value="x86_64"
    )

    assert client.build_user_agent(version="7.1.0") == (
        "snapcraft/7.1.0 Arch Linux/5.10.10-arch1-1 (x86_64)"
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


def test_get_hostname_none_is_unknown():
    assert client._get_hostname(hostname=None) == "UNKNOWN"


def test_get_hostname():
    assert client._get_hostname(hostname="acme") == "acme"


#######################
# StoreClient factory #
#######################


@pytest.mark.parametrize("ephemeral", (True, False))
def test_get_store_client(monkeypatch, ephemeral, legacy_config_path):
    monkeypatch.setenv("SNAPCRAFT_STORE_AUTH", "candid")
    legacy_config_path.unlink()

    store_client = client.get_client(ephemeral)

    assert isinstance(store_client, craft_store.StoreClient)


@pytest.mark.parametrize("ephemeral", (True, False))
def test_get_store_client_onprem(monkeypatch, ephemeral, legacy_config_path):
    monkeypatch.setenv("SNAPCRAFT_STORE_AUTH", "onprem")
    legacy_config_path.unlink()

    store_client = client.get_client(ephemeral)

    assert isinstance(store_client, client.OnPremClient)


@pytest.mark.parametrize("ephemeral", (True, False))
def test_get_ubuntu_client(ephemeral, legacy_config_path):
    legacy_config_path.unlink()

    store_client = client.get_client(ephemeral)

    assert isinstance(store_client, craft_store.UbuntuOneStoreClient)


@pytest.mark.parametrize("ephemeral", (True, False))
def test_get_legacy_ubuntu_client(new_dir, legacy_config_path, ephemeral):
    legacy_config_path.touch()

    store_client = client.get_client(ephemeral)

    if ephemeral:
        assert isinstance(store_client, craft_store.UbuntuOneStoreClient)
    else:
        assert isinstance(store_client, LegacyUbuntuOne)


########################
# LegacyStoreClientCLI #
########################


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
                status_code=requests.codes.unauthorized,
                content=json.dumps(
                    {"error_list": [{"message": "2fa", "code": "twofactor-required"}]}
                ).encode(),
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
def test_login_with_params(fake_client):
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


@pytest.mark.usefixtures("fake_client")
def test_login_with_env(monkeypatch):
    monkeypatch.setenv("SNAPCRAFT_STORE_CREDENTIALS", "secret")

    with pytest.raises(errors.SnapcraftError) as raised:
        client.StoreClientCLI().login(
            ttl=20,
            acls=["package_access", "package_push"],
            packages=["fake-snap", "fake-other-snap"],
            channels=["stable/fake", "edge/fake"],
        )

    assert str(raised.value) == "Cannot login with 'SNAPCRAFT_STORE_CREDENTIALS' set."
    assert raised.value.resolution == (
        "Unset 'SNAPCRAFT_STORE_CREDENTIALS' and try again."
    )


###########
# Request #
###########


@pytest.mark.usefixtures("fake_user_password", "fake_hostname")
def test_login_from_401_request(fake_client):
    fake_client.request.side_effect = [
        craft_store.errors.StoreServerError(
            FakeResponse(
                status_code=401,
                content=json.dumps(
                    {
                        "error_list": [
                            {
                                "code": "macaroon-needs-refresh",
                                "message": "Expired macaroon (age: 1234567 seconds)",
                            }
                        ]
                    }
                ).encode(),
            )
        ),
        FakeResponse(status_code=200, content=b"text"),
    ]

    client.StoreClientCLI().request("GET", "http://url.com/path")

    assert fake_client.request.mock_calls == [
        call("GET", "http://url.com/path"),
        call("GET", "http://url.com/path"),
    ]
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


def test_login_from_401_request_with_env_credentials(monkeypatch, fake_client):
    monkeypatch.setenv(constants.ENVIRONMENT_STORE_CREDENTIALS, "foo")
    fake_client.request.side_effect = [
        craft_store.errors.StoreServerError(
            FakeResponse(
                status_code=401,
                content=json.dumps(
                    {
                        "error_list": [
                            {
                                "code": "macaroon-needs-refresh",
                                "message": "Expired macaroon (age: 1234567 seconds)",
                            }
                        ]
                    }
                ).encode(),
            )
        ),
    ]

    with pytest.raises(errors.SnapcraftError) as raised:
        client.StoreClientCLI().request("GET", "http://url.com/path")

    assert str(raised.value) == (
        "Exported credentials are no longer valid for the Snap Store."
    )
    assert (
        raised.value.resolution
        == "Run export-login and update SNAPCRAFT_STORE_CREDENTIALS."
    )


def test_login_from_401_request_with_legacy_credentials(mocker, legacy_config_path):
    legacy_config_path.touch()
    mocker.patch(
        "snapcraft.store.client.LegacyUbuntuOne.request",
        side_effect=[
            craft_store.errors.StoreServerError(
                FakeResponse(
                    status_code=401,
                    content=json.dumps(
                        {
                            "error_list": [
                                {
                                    "code": "macaroon-needs-refresh",
                                    "message": "Expired macaroon (age: 1234567 seconds)",
                                }
                            ]
                        }
                    ).encode(),
                )
            ),
        ],
    )

    with pytest.raises(errors.SnapcraftError) as raised:
        client.StoreClientCLI().request("GET", "http://url.com/path")

    assert str(raised.value) == ("Credentials are no longer valid for the Snap Store.")
    assert (
        raised.value.resolution
        == "Run snapcraft login or export-login to obtain new credentials."
    )


############
# Register #
############


@pytest.mark.parametrize("private", [True, False])
@pytest.mark.parametrize("store_id", [None, "one-store", "other-store"])
def test_register(fake_client, private, store_id):
    client.StoreClientCLI().register("snap", is_private=private, store_id=store_id)

    expected_json = {
        "snap_name": "snap",
        "is_private": private,
        "series": "16",
    }
    if store_id:
        expected_json["store"] = store_id
    assert fake_client.request.mock_calls == [
        call(
            "POST",
            "https://dashboard.snapcraft.io/dev/api/register-name/",
            json=expected_json,
        )
    ]


###########################
# Get Account Information #
###########################


def test_get_account_info(fake_client):
    client.StoreClientCLI().get_account_info()

    assert fake_client.request.mock_calls == [
        call(
            "GET",
            "https://dashboard.snapcraft.io/dev/api/account",
            headers={"Accept": "application/json"},
        ),
        call().json(),
    ]


#########
# Names #
#########


def test_get_names(fake_client):
    fake_client.request.return_value = FakeResponse(
        status_code=200,
        content=json.dumps(
            {
                "snaps": {
                    "16": {
                        "test-snap-public": {
                            "private": False,
                            "since": "2016-07-26T20:18:32Z",
                            "status": "Approved",
                        },
                        "test-snap-private": {
                            "private": True,
                            "since": "2016-07-26T20:18:32Z",
                            "status": "Approved",
                        },
                        "test-snap-not-approved": {
                            "private": False,
                            "since": "2016-07-26T20:18:32Z",
                            "status": "Dispute",
                        },
                    }
                }
            },
        ).encode(),
    )

    assert client.StoreClientCLI().get_names() == [
        ("test-snap-public", "2016-07-26T20:18:32Z", "public", "-"),
        ("test-snap-private", "2016-07-26T20:18:32Z", "private", "-"),
    ]


###########
# Release #
###########


@pytest.mark.parametrize("progressive_percentage", [None, 100])
def test_release(fake_client, progressive_percentage):
    client.StoreClientCLI().release(
        snap_name="snap",
        revision=10,
        channels=["beta", "edge"],
        progressive_percentage=progressive_percentage,
    )

    assert fake_client.request.mock_calls == [
        call(
            "POST",
            "https://dashboard.snapcraft.io/dev/api/snap-release/",
            json={"name": "snap", "revision": "10", "channels": ["beta", "edge"]},
        )
    ]


def test_release_progressive(fake_client):
    client.StoreClientCLI().release(
        snap_name="snap",
        revision=10,
        channels=["beta", "edge"],
        progressive_percentage=88,
    )

    assert fake_client.request.mock_calls == [
        call(
            "POST",
            "https://dashboard.snapcraft.io/dev/api/snap-release/",
            json={
                "name": "snap",
                "revision": "10",
                "channels": ["beta", "edge"],
                "progressive": {"percentage": 88, "paused": False},
            },
        )
    ]


#########
# Close #
#########


def test_close(fake_client, monkeypatch):
    monkeypatch.setattr(
        client.StoreClientCLI,
        "get_account_info",
        lambda self: {
            "snaps": {constants.DEFAULT_SERIES: {"test-snap": {"snap-id": "12345"}}}
        },
    )

    client.StoreClientCLI().close(
        snap_name="test-snap",
        channel="edge",
    )

    assert fake_client.request.mock_calls == [
        call(
            "POST",
            "https://dashboard.snapcraft.io/dev/api/snaps/12345/close",
            json={"channels": ["edge"]},
        )
    ]


###################
# Get Channel Map #
###################


def test_get_channel_map(fake_client, channel_map_payload):
    fake_client.request.return_value = FakeResponse(
        status_code=200, content=json.dumps(channel_map_payload).encode()
    )
    channel_map = client.StoreClientCLI().get_channel_map(
        snap_name="test-snap",
    )
    assert isinstance(channel_map, ChannelMap)

    assert fake_client.request.mock_calls == [
        call(
            "GET",
            "https://dashboard.snapcraft.io/api/v2/snaps/test-snap/channel-map",
            headers={"Accept": "application/json"},
        )
    ]


#################
# Verify Upload #
#################


def test_verify_upload(fake_client):
    client.StoreClientCLI().verify_upload(snap_name="foo")

    assert fake_client.request.mock_calls == [
        call(
            "POST",
            "https://dashboard.snapcraft.io/dev/api/snap-push/",
            json={"name": "foo", "dry_run": True},
            headers={"Accept": "application/json"},
        )
    ]


#################
# Notify Upload #
#################


@pytest.mark.usefixtures("no_wait")
def test_notify_upload(fake_client):
    fake_client.request.side_effect = [
        FakeResponse(
            status_code=200,
            content=json.dumps({"status_details_url": "https://track"}).encode(),
        ),
        FakeResponse(
            status_code=200,
            content=json.dumps({"code": "processing", "processed": False}).encode(),
        ),
        FakeResponse(
            status_code=200,
            content=json.dumps(
                {"code": "done", "processed": True, "revision": 42}
            ).encode(),
        ),
    ]

    client.StoreClientCLI().notify_upload(
        snap_name="foo",
        upload_id="some-id",
        channels=None,
        built_at=None,
        snap_file_size=999,
        components=None,
    )

    assert fake_client.request.mock_calls == [
        call(
            "POST",
            "https://dashboard.snapcraft.io/dev/api/snap-push/",
            json={
                "name": "foo",
                "series": "16",
                "updown_id": "some-id",
                "binary_filesize": 999,
                "source_uploaded": False,
            },
            headers={"Accept": "application/json"},
        ),
        call("GET", "https://track"),
        call("GET", "https://track"),
    ]


@pytest.mark.usefixtures("no_wait")
def test_notify_upload_built_at(fake_client):
    fake_client.request.side_effect = [
        FakeResponse(
            status_code=200,
            content=json.dumps({"status_details_url": "https://track"}).encode(),
        ),
        FakeResponse(
            status_code=200,
            content=json.dumps({"code": "processing", "processed": False}).encode(),
        ),
        FakeResponse(
            status_code=200,
            content=json.dumps(
                {"code": "done", "processed": True, "revision": 42}
            ).encode(),
        ),
    ]

    client.StoreClientCLI().notify_upload(
        snap_name="foo",
        upload_id="some-id",
        channels=None,
        built_at="some-date",
        snap_file_size=999,
        components=None,
    )

    assert fake_client.request.mock_calls == [
        call(
            "POST",
            "https://dashboard.snapcraft.io/dev/api/snap-push/",
            json={
                "name": "foo",
                "series": "16",
                "updown_id": "some-id",
                "binary_filesize": 999,
                "source_uploaded": False,
                "built_at": "some-date",
            },
            headers={"Accept": "application/json"},
        ),
        call("GET", "https://track"),
        call("GET", "https://track"),
    ]


@pytest.mark.usefixtures("no_wait")
def test_notify_upload_channels(fake_client):
    fake_client.request.side_effect = [
        FakeResponse(
            status_code=200,
            content=json.dumps({"status_details_url": "https://track"}).encode(),
        ),
        FakeResponse(
            status_code=200,
            content=json.dumps({"code": "processing", "processed": False}).encode(),
        ),
        FakeResponse(
            status_code=200,
            content=json.dumps(
                {"code": "done", "processed": True, "revision": 42}
            ).encode(),
        ),
    ]

    client.StoreClientCLI().notify_upload(
        snap_name="foo",
        upload_id="some-id",
        channels=["stable"],
        built_at=None,
        snap_file_size=999,
        components=None,
    )

    assert fake_client.request.mock_calls == [
        call(
            "POST",
            "https://dashboard.snapcraft.io/dev/api/snap-push/",
            json={
                "name": "foo",
                "series": "16",
                "updown_id": "some-id",
                "binary_filesize": 999,
                "channels": ["stable"],
                "source_uploaded": False,
            },
            headers={"Accept": "application/json"},
        ),
        call("GET", "https://track"),
        call("GET", "https://track"),
    ]


@pytest.mark.usefixtures("no_wait")
def test_notify_upload_components(fake_client):
    fake_client.request.side_effect = [
        FakeResponse(
            status_code=200,
            content=json.dumps({"status_details_url": "https://track"}).encode(),
        ),
        FakeResponse(
            status_code=200,
            content=json.dumps({"code": "processing", "processed": False}).encode(),
        ),
        FakeResponse(
            status_code=200,
            content=json.dumps(
                {"code": "done", "processed": True, "revision": 42}
            ).encode(),
        ),
    ]

    client.StoreClientCLI().notify_upload(
        snap_name="foo",
        upload_id="some-id",
        channels=None,
        built_at=None,
        snap_file_size=999,
        components={"test-component": "test-upload-id"},
    )

    assert fake_client.request.mock_calls == [
        call(
            "POST",
            "https://dashboard.snapcraft.io/dev/api/snap-push/",
            json={
                "name": "foo",
                "series": "16",
                "updown_id": "some-id",
                "binary_filesize": 999,
                "source_uploaded": False,
                "components": {"test-component": "test-upload-id"},
            },
            headers={"Accept": "application/json"},
        ),
        call("GET", "https://track"),
        call("GET", "https://track"),
    ]


@pytest.mark.usefixtures("no_wait")
def test_notify_upload_error(fake_client):
    fake_client.request.side_effect = [
        FakeResponse(
            status_code=200,
            content=json.dumps({"status_details_url": "https://track"}).encode(),
        ),
        FakeResponse(
            status_code=200,
            content=json.dumps({"code": "processing", "processed": False}).encode(),
        ),
        FakeResponse(
            status_code=200,
            content=json.dumps(
                {"code": "done", "processed": True, "errors": [{"message": "bad-snap"}]}
            ).encode(),
        ),
    ]

    with pytest.raises(errors.SnapcraftError) as raised:
        client.StoreClientCLI().notify_upload(
            snap_name="foo",
            upload_id="some-id",
            channels=["stable"],
            built_at=None,
            snap_file_size=999,
            components=None,
        )

    assert str(raised.value) == textwrap.dedent(
        """\
        Issues while processing snap:
        - bad-snap"""
    )

    assert fake_client.request.mock_calls == [
        call(
            "POST",
            "https://dashboard.snapcraft.io/dev/api/snap-push/",
            json={
                "name": "foo",
                "series": "16",
                "updown_id": "some-id",
                "binary_filesize": 999,
                "channels": ["stable"],
                "source_uploaded": False,
            },
            headers={"Accept": "application/json"},
        ),
        call("GET", "https://track"),
        call("GET", "https://track"),
    ]


##################
# List Revisions #
##################


def test_list_revisions(fake_client, list_revisions_payload):
    fake_client.request.return_value = FakeResponse(
        status_code=200, content=json.dumps(list_revisions_payload).encode()
    )
    channel_map = client.StoreClientCLI().list_revisions(
        snap_name="test-snap",
    )
    assert isinstance(channel_map, Releases)

    assert fake_client.request.mock_calls == [
        call(
            "GET",
            "https://dashboard.snapcraft.io/api/v2/snaps/test-snap/releases",
            headers={"Content-Type": "application/json", "Accept": "application/json"},
        )
    ]


#######################
# List Confdb Schemas #
#######################


@pytest.mark.parametrize("name", [None, "test-confdb"])
def test_list_confdb_schemas(name, fake_client, list_confdb_schemas_payload, check):
    """Test the list confdb schemas endpoint."""
    fake_client.request.return_value = FakeResponse(
        status_code=200, content=json.dumps(list_confdb_schemas_payload).encode()
    )

    confdb_schemas = client.StoreClientCLI().list_confdb_schemas(name=name)

    check.is_instance(confdb_schemas, list)
    for confdb_schema in confdb_schemas:
        check.is_instance(confdb_schema, models.ConfdbSchemaAssertion)
        check.equal(
            confdb_schema.body,
            '{\n  "storage": {\n    "schema": {\n      "wifi": {\n        '
            '"values": "any"\n      }\n    }\n  }\n}',
        )
    check.equal(
        fake_client.request.mock_calls,
        [
            call(
                "GET",
                f"https://dashboard.snapcraft.io/api/v2/confdb-schemas{f'/{name}' if name else ''}",
                headers={
                    "Content-Type": "application/json",
                    "Accept": "application/json",
                },
            )
        ],
    )


def test_list_confdb_schemas_empty(fake_client, check):
    """Test the list confdb schemas endpoint with nothing returned."""
    fake_client.request.return_value = FakeResponse(
        status_code=200, content=json.dumps({"assertions": []}).encode()
    )

    confdb_schemas = client.StoreClientCLI().list_confdb_schemas()

    check.equal(confdb_schemas, [])
    check.equal(
        fake_client.request.mock_calls,
        [
            call(
                "GET",
                "https://dashboard.snapcraft.io/api/v2/confdb-schemas",
                headers={
                    "Content-Type": "application/json",
                    "Accept": "application/json",
                },
            )
        ],
    )


def test_list_confdb_schemas_unmarshal_error(fake_client, list_confdb_schemas_payload):
    """Raise an error if the response cannot be unmarshalled."""
    list_confdb_schemas_payload["assertions"][0]["headers"].pop("name")
    fake_client.request.return_value = FakeResponse(
        status_code=200, content=json.dumps(list_confdb_schemas_payload).encode()
    )

    with pytest.raises(errors.SnapcraftAssertionError) as raised:
        client.StoreClientCLI().list_confdb_schemas()

    assert str(raised.value) == "Received invalid confdb schema from the store"
    assert raised.value.details == (
        "Bad confdb schema content:\n- field 'name' required in top-level configuration"
    )


#######################
# Build Confdb Schema #
#######################


def test_build_confdb_schema(fake_client, build_confdb_schema_payload):
    """Test the build confdb schema endpoint."""
    mock_confdb_schema = Mock(spec=models.ConfdbSchemaAssertion)
    expected_confdb_schema = models.ConfdbSchemaAssertion(**build_confdb_schema_payload)
    fake_client.request.return_value = FakeResponse(
        status_code=200, content=json.dumps(build_confdb_schema_payload).encode()
    )

    confdb_schema = client.StoreClientCLI().build_confdb_schema(
        confdb_schema=mock_confdb_schema
    )

    assert confdb_schema == expected_confdb_schema
    assert fake_client.request.mock_calls == [
        call(
            "POST",
            "https://dashboard.snapcraft.io/api/v2/confdb-schemas/build-assertion",
            headers={
                "Content-Type": "application/json",
                "Accept": "application/json",
            },
            json=mock_confdb_schema.marshal(),
        )
    ]


def test_build_confdb_schema_unmarshal_error(fake_client, build_confdb_schema_payload):
    """Raise an error if the response cannot be unmarshalled."""
    mock_confdb_schema = Mock(spec=models.ConfdbSchemaAssertion)
    build_confdb_schema_payload.pop("name")
    fake_client.request.return_value = FakeResponse(
        status_code=200, content=json.dumps(build_confdb_schema_payload).encode()
    )

    with pytest.raises(errors.SnapcraftAssertionError) as raised:
        client.StoreClientCLI().build_confdb_schema(confdb_schema=mock_confdb_schema)

    assert str(raised.value) == "Received invalid confdb schema from the store"
    assert raised.value.details == (
        "Bad confdb schema content:\n- field 'name' required in top-level configuration"
    )


######################
# Post Confdb Schema #
######################


def test_post_confdb_schema(fake_client, post_confdb_schema_payload):
    """Test the post confdb schema endpoint."""
    expected_confdb_schema = models.ConfdbSchemaAssertion(
        **post_confdb_schema_payload["assertions"][0]["headers"],
        body=post_confdb_schema_payload["assertions"][0]["body"],
    )
    fake_client.request.return_value = FakeResponse(
        status_code=200, content=json.dumps(post_confdb_schema_payload).encode()
    )

    confdb_schema = client.StoreClientCLI().post_confdb_schema(
        confdb_schema_data=b"test-data"
    )

    assert confdb_schema == expected_confdb_schema
    assert fake_client.request.mock_calls == [
        call(
            "POST",
            "https://dashboard.snapcraft.io/api/v2/confdb-schemas",
            headers={
                "Accept": "application/json",
                "Content-Type": "application/x.ubuntu.assertion",
            },
            data=b"test-data",
        )
    ]


@pytest.mark.parametrize("num_assertions", [0, 2])
def test_post_confdb_schema_wrong_payload_error(
    num_assertions, fake_client, post_confdb_schema_payload
):
    """Error if the wrong number of assertions are returned."""
    post_confdb_schema_payload["assertions"] = (
        post_confdb_schema_payload["assertions"] * num_assertions
    )
    fake_client.request.return_value = FakeResponse(
        status_code=200, content=json.dumps(post_confdb_schema_payload).encode()
    )

    with pytest.raises(errors.SnapcraftAssertionError) as raised:
        client.StoreClientCLI().post_confdb_schema(confdb_schema_data=b"test-data")

    assert str(raised.value) == "Received invalid confdb schema from the store"


def test_post_confdb_schema_unmarshal_error(fake_client, post_confdb_schema_payload):
    """Raise an error if the response cannot be unmarshalled."""
    post_confdb_schema_payload["assertions"][0]["headers"].pop("name")
    fake_client.request.return_value = FakeResponse(
        status_code=200, content=json.dumps(post_confdb_schema_payload).encode()
    )

    with pytest.raises(errors.SnapcraftAssertionError) as raised:
        client.StoreClientCLI().post_confdb_schema(confdb_schema_data=b"test-data")

    assert str(raised.value) == "Received invalid confdb schema from the store"
    assert raised.value.details == (
        "Bad confdb schema content:\n- field 'name' required in top-level configuration"
    )


########################
# List Validation Sets #
########################


@pytest.mark.parametrize("name", [None, "test-validation-set"])
@pytest.mark.parametrize("sequence", [None, 123])
def test_list_validation_sets(
    name, sequence, fake_client, list_validation_sets_payload, check
):
    """Test the list validation sets endpoint."""
    fake_client.request.return_value = FakeResponse(
        status_code=200, content=json.dumps(list_validation_sets_payload).encode()
    )

    validation_sets = client.StoreClientCLI().list_validation_sets(
        name=name, sequence=sequence
    )

    check.is_instance(validation_sets, list)
    for validation_set in validation_sets:
        check.is_instance(validation_set, models.ValidationSetAssertion)
    check.equal(
        fake_client.request.mock_calls,
        [
            call(
                "GET",
                f"https://dashboard.snapcraft.io/api/v2/validation-sets{f'/{name}' if name else ''}",
                headers={
                    "Content-Type": "application/json",
                    "Accept": "application/json",
                },
                params={"sequence": sequence} if sequence else {},
            )
        ],
    )


def test_list_validation_sets_empty(fake_client, check):
    """Test the list validation sets endpoint with nothing returned."""
    fake_client.request.return_value = FakeResponse(
        status_code=200, content=json.dumps({"assertions": []}).encode()
    )

    validation_sets = client.StoreClientCLI().list_validation_sets()

    check.equal(validation_sets, [])
    check.equal(
        fake_client.request.mock_calls,
        [
            call(
                "GET",
                "https://dashboard.snapcraft.io/api/v2/validation-sets",
                headers={
                    "Content-Type": "application/json",
                    "Accept": "application/json",
                },
                params={},
            )
        ],
    )


def test_list_validation_sets_unmarshal_error(
    fake_client, list_validation_sets_payload
):
    """Raise an error if the response cannot be unmarshalled."""
    list_validation_sets_payload["assertions"][0]["headers"].pop("name")
    fake_client.request.return_value = FakeResponse(
        status_code=200, content=json.dumps(list_validation_sets_payload).encode()
    )

    with pytest.raises(errors.SnapcraftAssertionError) as raised:
        client.StoreClientCLI().list_validation_sets()

    assert str(raised.value) == "Received invalid validation set from the store"
    assert raised.value.details == (
        "Bad validation set content:\n- field 'name' required in top-level configuration"
    )


########################
# Build Validation Set #
########################


def test_build_validation_set(fake_client, build_validation_set_payload):
    """Test the build validation set endpoint."""
    mock_validation_set = Mock(spec=models.ValidationSetAssertion)
    expected_validation_set = models.ValidationSetAssertion(
        **build_validation_set_payload
    )
    fake_client.request.return_value = FakeResponse(
        status_code=200, content=json.dumps(build_validation_set_payload).encode()
    )

    validation_set = client.StoreClientCLI().build_validation_set(
        validation_set=mock_validation_set
    )

    assert validation_set == expected_validation_set
    assert fake_client.request.mock_calls == [
        call(
            "POST",
            "https://dashboard.snapcraft.io/api/v2/validation-sets/build-assertion",
            headers={
                "Content-Type": "application/json",
                "Accept": "application/json",
            },
            json=mock_validation_set.marshal(),
        )
    ]


def test_build_validation_set_unmarshal_error(
    fake_client, build_validation_set_payload
):
    """Raise an error if the response cannot be unmarshalled."""
    mock_validation_set = Mock(spec=models.ValidationSetAssertion)
    build_validation_set_payload.pop("name")
    fake_client.request.return_value = FakeResponse(
        status_code=200, content=json.dumps(build_validation_set_payload).encode()
    )

    with pytest.raises(errors.SnapcraftAssertionError) as raised:
        client.StoreClientCLI().build_validation_set(validation_set=mock_validation_set)

    assert str(raised.value) == "Received invalid validation set from the store"
    assert raised.value.details == (
        "Bad validation set content:\n- field 'name' required in top-level configuration"
    )


#######################
# Post Validation Set #
#######################


def test_post_validation_set(fake_client, post_validation_set_payload):
    """Test the post validation set endpoint."""
    expected_validation_set = models.ValidationSetAssertion(
        **post_validation_set_payload["assertions"][0]["headers"],
    )
    fake_client.request.return_value = FakeResponse(
        status_code=200, content=json.dumps(post_validation_set_payload).encode()
    )

    validation_set = client.StoreClientCLI().post_validation_set(
        validation_set_data=b"test-data"
    )

    assert validation_set == expected_validation_set
    assert fake_client.request.mock_calls == [
        call(
            "POST",
            "https://dashboard.snapcraft.io/api/v2/validation-sets",
            headers={
                "Accept": "application/json",
                "Content-Type": "application/x.ubuntu.assertion",
            },
            data=b"test-data",
        )
    ]


@pytest.mark.parametrize("num_assertions", [0, 2])
def test_post_validation_set_wrong_payload_error(
    num_assertions, fake_client, post_validation_set_payload
):
    """Error if the wrong number of assertions are returned."""
    post_validation_set_payload["assertions"] = (
        post_validation_set_payload["assertions"] * num_assertions
    )
    fake_client.request.return_value = FakeResponse(
        status_code=200, content=json.dumps(post_validation_set_payload).encode()
    )

    with pytest.raises(errors.SnapcraftAssertionError) as raised:
        client.StoreClientCLI().post_validation_set(validation_set_data=b"test-data")

    assert str(raised.value) == "Received invalid validation set from the store"


def test_post_validation_set_unmarshal_error(fake_client, post_validation_set_payload):
    """Raise an error if the response cannot be unmarshalled."""
    post_validation_set_payload["assertions"][0]["headers"].pop("name")
    fake_client.request.return_value = FakeResponse(
        status_code=200, content=json.dumps(post_validation_set_payload).encode()
    )

    with pytest.raises(errors.SnapcraftAssertionError) as raised:
        client.StoreClientCLI().post_validation_set(validation_set_data=b"test-data")

    assert str(raised.value) == "Received invalid validation set from the store"
    assert raised.value.details == (
        "Bad validation set content:\n- field 'name' required in top-level configuration"
    )


########################
# OnPremStoreClientCLI #
########################


@pytest.fixture
def fake_client_request(mocker):
    return mocker.patch("snapcraft.store.client.OnPremClient.request", autospec=True)


@pytest.fixture
def fake_client_notify_revision(mocker):
    return mocker.patch(
        "snapcraft.store.client.OnPremClient.notify_revision",
        autospec=True,
        return_value=RevisionsResponseModel.unmarshal(
            {"status-url": "https://status.com/fake"}
        ),
    )


@pytest.fixture
def on_prem_client(monkeypatch):
    monkeypatch.setenv("SNAPCRAFT_STORE_AUTH", "onprem")
    # Remove any sleep calls from the client.
    monkeypatch.setattr("time.sleep", lambda x: x)
    return client.StoreClientCLI()


def test_onprem_request(on_prem_client, fake_client_request):
    on_prem_client.request("GET", "https://foo.bar")

    assert fake_client_request.mock_calls == [call(ANY, "GET", "https://foo.bar")]


def test_on_prem_verify_upload(on_prem_client, emitter):
    on_prem_client.verify_upload(snap_name="fake-snap")

    emitter.assert_debug("Skipping verification for 'fake-snap'")


def test_on_prem_notify_revision_release_unsupported(on_prem_client):
    with pytest.raises(errors.SnapcraftError):
        on_prem_client.notify_upload(
            snap_name="fake-snap",
            upload_id="fake-id",
            snap_file_size=10,
            built_at=None,
            channels=["stable"],
            components=None,
        )


def test_on_prem_notify_upload_components_unsupported(on_prem_client):
    with pytest.raises(errors.SnapcraftError) as raised:
        on_prem_client.notify_upload(
            snap_name="fake-snap",
            upload_id="fake-id",
            snap_file_size=10,
            built_at=None,
            channels=None,
            components={"test-component": "test-upload-id"},
        )

    assert (
        str(raised.value) == "Components are currently unsupported for on-prem stores"
    )


@pytest.mark.usefixtures("fake_client_notify_revision")
def test_on_prem_notify_revision_approved(on_prem_client, fake_client_request, emitter):
    fake_client_request.side_effect = [
        FakeResponse(
            content=json.dumps({"revisions": [{"status": "progress"}]}).encode(),
            status_code=200,
        ),
        FakeResponse(
            content=json.dumps(
                {"revisions": [{"status": "approved", "revision": 2}]}
            ).encode(),
            status_code=200,
        ),
    ]

    assert (
        on_prem_client.notify_upload(
            snap_name="fake-snap",
            upload_id="fake-id",
            snap_file_size=10,
            built_at=None,
            channels=None,
            components=None,
        )
        == 2
    )

    emitter.assert_debug("Ignoring snap_file_size of 10 and built_at None")


@pytest.mark.usefixtures("fake_client_notify_revision")
def test_on_prem_notify_revision_rejected(on_prem_client, fake_client_request):
    fake_client_request.side_effect = [
        FakeResponse(
            content=json.dumps({"revisions": [{"status": "progress"}]}).encode(),
            status_code=200,
        ),
        FakeResponse(
            content=json.dumps(
                {
                    "revisions": [
                        {
                            "status": "rejected",
                            "errors": [
                                {"code": "bad-snap-code", "message": "bad snap"}
                            ],
                        }
                    ]
                }
            ).encode(),
            status_code=200,
        ),
    ]

    with pytest.raises(errors.SnapcraftError) as raised:
        on_prem_client.notify_upload(
            snap_name="fake-snap",
            upload_id="fake-id",
            snap_file_size=10,
            built_at=None,
            channels=None,
            components=None,
        )

    assert str(raised.value) == "Error uploading snap: bad-snap-code"


def test_on_prem_release(on_prem_client, fake_client_request):
    on_prem_client.release("fake-snap", revision=1, channels=["stable", "edge"])

    assert fake_client_request.mock_calls == [
        call(
            ANY,
            "POST",
            "https://dashboard.snapcraft.io/v1/snap/fake-snap/releases",
            json=[
                {"revision": 1, "channel": "stable"},
                {"revision": 1, "channel": "edge"},
            ],
        )
    ]


def test_on_prem_release_progressive_percentage_unsupported(on_prem_client):
    with pytest.raises(errors.SnapcraftError):
        on_prem_client.release(
            "fake-snap",
            revision=1,
            channels=["stable", "edge"],
            progressive_percentage=10,
        )


def test_on_prem_close(on_prem_client, fake_client_request):
    on_prem_client.close("fake-snap", channel="stable")

    assert fake_client_request.mock_calls == [
        call(
            ANY,
            "POST",
            "https://dashboard.snapcraft.io/v1/snap/fake-snap/releases",
            json=[
                {"revision": None, "channel": "stable"},
            ],
        )
    ]


def test_on_prem_get_channel_map(
    on_prem_client, fake_client_request, channel_map_payload
):
    extra_revision_data = {
        "confinement": "strict",
        "created-at": "2020-02-03T20:58:37Z",
        "created-by": "QQPbzX6aaSF7ckKU5tGWnwfai1C4tiJu",
        "grade": "stable",
        "revision": 1,
        "sha3-384": "short-sha3-384",
        "size": 20480,
        "status": "released",
        "type": "app",
        "version": "6.4",
    }
    channel_map_payload["revisions"][0].update(extra_revision_data)
    channel_map_payload["package"] = channel_map_payload.pop("snap")

    fake_client_request.return_value = FakeResponse(
        status_code=200, content=json.dumps(channel_map_payload).encode()
    )
    channel_map = on_prem_client.get_channel_map(
        snap_name="test-snap",
    )
    assert isinstance(channel_map, ChannelMap)

    assert fake_client_request.mock_calls == [
        call(
            ANY,
            "GET",
            "https://dashboard.snapcraft.io/v1/snap/test-snap/releases",
        )
    ]


def test_on_prem_list_revisions(
    on_prem_client, fake_client_request, list_revisions_payload
):
    fake_client_request.return_value = FakeResponse(
        status_code=200, content=json.dumps(list_revisions_payload).encode()
    )
    channel_map = client.StoreClientCLI().list_revisions(
        snap_name="test-snap",
    )
    assert isinstance(channel_map, Releases)

    assert fake_client_request.mock_calls == [
        call(
            ANY,
            "GET",
            "https://dashboard.snapcraft.io/v1/snap/test-snap/revisions",
            headers={"Content-Type": "application/json", "Accept": "application/json"},
        )
    ]
