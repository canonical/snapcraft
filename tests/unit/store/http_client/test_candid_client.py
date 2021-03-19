# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2021 Canonical Ltd
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

import io
import json
from textwrap import dedent
from unittest.mock import Mock, call, patch

import pytest
from macaroonbakery import bakery, httpbakery
from pymacaroons.macaroon import Macaroon

from snapcraft.storeapi.http_clients._candid_client import (
    CandidClient,
    CandidConfig,
    _http_client,
)


def test_config_section_name():
    assert CandidConfig()._get_section_name() == "dashboard.snapcraft.io"


def test_config_section_name_with_env(monkeypatch):
    monkeypatch.setenv("STORE_DASHBOARD_URL", "http://dashboard.other.com")

    assert CandidConfig()._get_section_name() == "dashboard.other.com"


def test_config_path(xdg_dirs):
    assert (
        CandidConfig()._get_config_path() == xdg_dirs / ".config/snapcraft/candid.cfg"
    )


def test_candid_client_has_no_credentials(xdg_dirs):
    assert CandidClient().has_credentials() is False


def test_candid_client_has_credentials(xdg_dirs):
    # Baseline check.
    assert CandidClient().has_credentials() is False

    # Setup.
    client = CandidClient()
    client._macaroon = "macaroon"
    client._auth = "auth"

    assert client.has_credentials() is True
    assert CandidClient().has_credentials() is True


@pytest.fixture
def candid_client(xdg_dirs, monkeypatch):
    """Return a CandidClient with an alterate requests method."""
    bakery_client = Mock(spec=httpbakery.Client)

    def mock_discharge(*args, **kwargs):
        return [
            Macaroon(
                location="api.snapcraft.io",
                signature="d9533461d7835e4851c7e3b639144406cf768597dea6e133232fbd2385a5c050",
            )
        ]

    monkeypatch.setattr(bakery, "discharge_all", mock_discharge)

    return CandidClient(bakery_client=bakery_client)


@pytest.fixture
def snapcraft_macaroon():
    return json.dumps(
        {
            "s64": "a0Vi7CwhHWjS4bxzKPhCZQIEJDvlbh9FyhOtWx0tNFQ",
            "c": [
                {"i": "time-before 2022-03-18T19:54:57.151721Z"},
                {
                    "v64": "pDqaL9KDrPfCQCLDUdPc8yO2bTQheWGsM1tpxRaS_4BT3r6zpdnT5TelXz8vpjb4iUhTnc60-x5DPKJOpRuwAi4qMdNa67Vo",
                    "l": "https://api.jujucharms.com/identity/",
                    "i64": "AoZh2j7mbDQgh3oK3qMqoXKKFAnJvmOKwmDCNYHIxHqQnFLJZJUBpqoiJtqra-tyXPPMUTmfuXMgOWP7xKwTD26FBgtJBdh1mE1wt3kf0Ur_TnOzbAWQCHKxqK9jAp1jYv-LlLLAlQAmoqvz9fBf2--dIxHiLIRTThmAESAnlLZHOJ7praDmIScsLQC475a85avA",
                },
                {
                    "i": 'extra {"package_id": null, "channel": null, "acl": ["package_access", "package_manage", "package_push", "package_register", "package_release", "package_update"], "store_ids": null}'
                },
            ],
            "l": "api.snapcraft.io",
            "i64": "AwoQ2Ft5YBjnovqdr8VNV3TSlhIBMBoOCgVsb2dpbhIFbG9naW4",
        }
    )


def test_login_discharge_macaroon(candid_client, snapcraft_macaroon):
    candid_client.request
    candid_client.login(macaroon=snapcraft_macaroon)

    assert candid_client.has_credentials() is True
    assert candid_client._macaroon == snapcraft_macaroon
    assert candid_client._auth == (
        "W3siaWRlbnRpZmllciI6ICIiLCAic2lnbmF0dXJlIjogImQ5NTMzNDYxZDc4MzVlNDg1MWM"
        "3ZTNiNjM5MTQ0NDA2Y2Y3Njg1OTdkZWE2ZTEzMzIzMmZiZDIzODVhNWMwNTAiLCAibG9jYX"
        "Rpb24iOiAiYXBpLnNuYXBjcmFmdC5pbyJ9XQ=="
    )


def test_login_discharge_macaroon_no_save(candid_client, snapcraft_macaroon):
    candid_client.login(macaroon=snapcraft_macaroon, save=False)

    assert candid_client.has_credentials() is False
    assert candid_client._macaroon == snapcraft_macaroon
    assert candid_client._auth == (
        "W3siaWRlbnRpZmllciI6ICIiLCAic2lnbmF0dXJlIjogImQ5NTMzNDYxZDc4MzVlNDg1MWM"
        "3ZTNiNjM5MTQ0NDA2Y2Y3Njg1OTdkZWE2ZTEzMzIzMmZiZDIzODVhNWMwNTAiLCAibG9jYX"
        "Rpb24iOiAiYXBpLnNuYXBjcmFmdC5pbyJ9XQ=="
    )


def test_login_with_config_fd(candid_client, snapcraft_macaroon):
    with io.StringIO() as config_fd:
        print("[dashboard.snapcraft.io]", file=config_fd)
        print(f"macaroon = {snapcraft_macaroon}", file=config_fd)
        print(f"auth = 1234567890noshare", file=config_fd)
        config_fd.seek(0)

        candid_client.login(config_fd=config_fd)

    assert candid_client.has_credentials() is True
    assert candid_client._macaroon == snapcraft_macaroon
    assert candid_client._auth == "1234567890noshare"


def test_login_with_config_fd_no_save(candid_client, snapcraft_macaroon):
    with io.StringIO() as config_fd:
        print("[dashboard.snapcraft.io]", file=config_fd)
        print(f"macaroon = {snapcraft_macaroon}", file=config_fd)
        print(f"auth = 1234567890noshare", file=config_fd)
        config_fd.seek(0)

        candid_client.login(config_fd=config_fd, save=False)

    assert candid_client.has_credentials() is False
    assert candid_client._macaroon == snapcraft_macaroon
    assert candid_client._auth == "1234567890noshare"


@pytest.fixture
def authed_client(candid_client, snapcraft_macaroon):
    candid_client.login(macaroon=snapcraft_macaroon)
    assert candid_client.has_credentials() is True

    return candid_client


def test_logout(authed_client):
    authed_client.logout()

    assert authed_client.has_credentials() is False


def test_export_login(authed_client):
    with io.StringIO() as config_fd:
        authed_client.export_login(config_fd=config_fd, encode=False)

        config_fd.seek(0)

        assert config_fd.getvalue().strip() == dedent(
            f"""\
        [dashboard.snapcraft.io]
        auth = {authed_client._auth}
        macaroon = {authed_client._macaroon}"""
        )


def test_export_login_base64_encoded(authed_client):
    with io.StringIO() as config_fd:
        authed_client.export_login(config_fd=config_fd, encode=True)

        config_fd.seek(0)

        assert config_fd.getvalue().strip() == (
            "W2Rhc2hib2FyZC5zbmFwY3JhZnQuaW9dCmF1dGggPSBXM3NpYVdSbGJuUnBabWxsY2lJNklDSWlMQ0FpY"
            "zJsbmJtRjBkWEpsSWpvZ0ltUTVOVE16TkRZeFpEYzRNelZsTkRnMU1XTTNaVE5pTmpNNU1UUTBOREEyWT"
            "JZM05qZzFPVGRrWldFMlpURXpNekl6TW1aaVpESXpPRFZoTldNd05UQWlMQ0FpYkc5allYUnBiMjRpT2l"
            "BaVlYQnBMbk51WVhCamNtRm1kQzVwYnlKOVhRPT0KbWFjYXJvb24gPSB7InM2NCI6ICJhMFZpN0N3aEhX"
            "alM0Ynh6S1BoQ1pRSUVKRHZsYmg5RnloT3RXeDB0TkZRIiwgImMiOiBbeyJpIjogInRpbWUtYmVmb3JlI"
            "DIwMjItMDMtMThUMTk6NTQ6NTcuMTUxNzIxWiJ9LCB7InY2NCI6ICJwRHFhTDlLRHJQZkNRQ0xEVWRQYz"
            "h5TzJiVFFoZVdHc00xdHB4UmFTXzRCVDNyNnpwZG5UNVRlbFh6OHZwamI0aVVoVG5jNjAteDVEUEtKT3B"
            "SdXdBaTRxTWROYTY3Vm8iLCAibCI6ICJodHRwczovL2FwaS5qdWp1Y2hhcm1zLmNvbS9pZGVudGl0eS8i"
            "LCAiaTY0IjogIkFvWmgyajdtYkRRZ2gzb0szcU1xb1hLS0ZBbkp2bU9Ld21EQ05ZSEl4SHFRbkZMSlpKV"
            "UJwcW9pSnRxcmEtdHlYUFBNVVRtZnVYTWdPV1A3eEt3VEQyNkZCZ3RKQmRoMW1FMXd0M2tmMFVyX1RuT3"
            "piQVdRQ0hLeHFLOWpBcDFqWXYtTGxMTEFsUUFtb3F2ejlmQmYyLS1kSXhIaUxJUlRUaG1BRVNBbmxMWkh"
            "PSjdwcmFEbUlTY3NMUUM0NzVhODVhdkEifSwgeyJpIjogImV4dHJhIHtcInBhY2thZ2VfaWRcIjogbnVs"
            "bCwgXCJjaGFubmVsXCI6IG51bGwsIFwiYWNsXCI6IFtcInBhY2thZ2VfYWNjZXNzXCIsIFwicGFja2FnZ"
            "V9tYW5hZ2VcIiwgXCJwYWNrYWdlX3B1c2hcIiwgXCJwYWNrYWdlX3JlZ2lzdGVyXCIsIFwicGFja2FnZV"
            "9yZWxlYXNlXCIsIFwicGFja2FnZV91cGRhdGVcIl0sIFwic3RvcmVfaWRzXCI6IG51bGx9In1dLCAibCI"
            "6ICJhcGkuc25hcGNyYWZ0LmlvIiwgImk2NCI6ICJBd29RMkZ0NVlCam5vdnFkcjhWTlYzVFNsaElCTUJv"
            "T0NnVnNiMmRwYmhJRmJHOW5hVzQifQoK"
        )


@pytest.fixture
def request_mock():
    patched = patch.object(
        _http_client.Client, "request", spec=_http_client.Client.request
    )
    try:
        yield patched.start()
    finally:
        patched.stop()


@pytest.mark.parametrize("method", ["GET", "PUT", "POST"])
@pytest.mark.parametrize("params", [None, {}, {"foo": "bar"}])
def test_request(authed_client, request_mock, method, params):
    authed_client.request(method, "https://dashboard.snapcraft.io/foo", params=params)

    assert request_mock.mock_calls == [
        call(
            method,
            "https://dashboard.snapcraft.io/foo",
            params=params,
            headers={"Macaroons": authed_client._auth},
        ),
        call().ok.__bool__(),
    ]


def test_request_with_headers(authed_client, request_mock):
    authed_client.request(
        "GET", "https://dashboard.snapcraft.io/foo", headers={"foo": "bar"}
    )

    assert request_mock.mock_calls == [
        call(
            "GET",
            "https://dashboard.snapcraft.io/foo",
            params=None,
            headers={"foo": "bar", "Macaroons": authed_client._auth},
        ),
        call().ok.__bool__(),
    ]


@pytest.mark.parametrize("method", ["GET", "PUT", "POST"])
@pytest.mark.parametrize("params", [None, {}, {"foo": "bar"}])
@pytest.mark.parametrize("headers", [None, {}, {"foo": "bar"}])
def test_request_no_auth(authed_client, request_mock, method, params, headers):
    authed_client.request(
        method,
        "https://dashboard.snapcraft.io/foo",
        params=params,
        headers=headers,
        auth_header=False,
    )

    assert request_mock.mock_calls == [
        call(
            method, "https://dashboard.snapcraft.io/foo", params=params, headers=headers
        ),
        call().ok.__bool__(),
    ]
