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
from unittest.mock import ANY, call

import craft_store
import pytest
import requests
from craft_store import endpoints
from craft_store.models import RevisionsResponseModel

from snapcraft import errors
from snapcraft.store import LegacyUbuntuOne, client, constants
from snapcraft.store.channel_map import ChannelMap
from snapcraft.utils import OSPlatform
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
                ),
            )
        ),
        FakeResponse(status_code=200, content="text"),
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
                ),
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
                    ),
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
        ),
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
        status_code=200, content=json.dumps(channel_map_payload)
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
            status_code=200, content=json.dumps({"status_details_url": "https://track"})
        ),
        FakeResponse(
            status_code=200,
            content=json.dumps({"code": "processing", "processed": False}),
        ),
        FakeResponse(
            status_code=200,
            content=json.dumps({"code": "done", "processed": True, "revision": 42}),
        ),
    ]

    client.StoreClientCLI().notify_upload(
        snap_name="foo",
        upload_id="some-id",
        channels=None,
        built_at=None,
        snap_file_size=999,
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
            status_code=200, content=json.dumps({"status_details_url": "https://track"})
        ),
        FakeResponse(
            status_code=200,
            content=json.dumps({"code": "processing", "processed": False}),
        ),
        FakeResponse(
            status_code=200,
            content=json.dumps({"code": "done", "processed": True, "revision": 42}),
        ),
    ]

    client.StoreClientCLI().notify_upload(
        snap_name="foo",
        upload_id="some-id",
        channels=None,
        built_at="some-date",
        snap_file_size=999,
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
            status_code=200, content=json.dumps({"status_details_url": "https://track"})
        ),
        FakeResponse(
            status_code=200,
            content=json.dumps({"code": "processing", "processed": False}),
        ),
        FakeResponse(
            status_code=200,
            content=json.dumps({"code": "done", "processed": True, "revision": 42}),
        ),
    ]

    client.StoreClientCLI().notify_upload(
        snap_name="foo",
        upload_id="some-id",
        channels=["stable"],
        built_at=None,
        snap_file_size=999,
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
def test_notify_upload_error(fake_client):
    fake_client.request.side_effect = [
        FakeResponse(
            status_code=200, content=json.dumps({"status_details_url": "https://track"})
        ),
        FakeResponse(
            status_code=200,
            content=json.dumps({"code": "processing", "processed": False}),
        ),
        FakeResponse(
            status_code=200,
            content=json.dumps(
                {"code": "done", "processed": True, "errors": [{"message": "bad-snap"}]}
            ),
        ),
    ]

    with pytest.raises(errors.SnapcraftError) as raised:
        client.StoreClientCLI().notify_upload(
            snap_name="foo",
            upload_id="some-id",
            channels=["stable"],
            built_at=None,
            snap_file_size=999,
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
        status_code=200, content=json.dumps(list_revisions_payload)
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
        )


@pytest.mark.usefixtures("fake_client_notify_revision")
def test_on_prem_notify_revision_approved(on_prem_client, fake_client_request, emitter):
    fake_client_request.side_effect = [
        FakeResponse(
            content=json.dumps({"revisions": [{"status": "progress"}]}), status_code=200
        ),
        FakeResponse(
            content=json.dumps({"revisions": [{"status": "approved", "revision": 2}]}),
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
        )
        == 2
    )

    emitter.assert_debug("Ignoring snap_file_size of 10 and built_at None")


@pytest.mark.usefixtures("fake_client_notify_revision")
def test_on_prem_notify_revision_rejected(on_prem_client, fake_client_request):
    fake_client_request.side_effect = [
        FakeResponse(
            content=json.dumps({"revisions": [{"status": "progress"}]}), status_code=200
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
            ),
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
        status_code=200, content=json.dumps(channel_map_payload)
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
        status_code=200, content=json.dumps(list_revisions_payload)
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
