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

"""On premises Store Client unit tests."""

from unittest.mock import call

import pytest

from snapcraft import errors
from snapcraft.store import constants
from snapcraft.store.onprem_client import ON_PREM_ENDPOINTS, OnPremClient


@pytest.fixture
def on_prem_client():
    """Return an instance of the OnPremClient"""
    return OnPremClient(
        base_url="https://fake-store.io",
        storage_base_url="",
        endpoints=ON_PREM_ENDPOINTS,
        application_name="onprem-test",
        user_agent="agent",
        environment_auth=constants.ENVIRONMENT_STORE_CREDENTIALS,
        ephemeral=False,
    )


def test_get_macaroon(on_prem_client, monkeypatch):
    """Retrieve the admin macaroon from the environment."""
    monkeypatch.setenv(constants.ENVIRONMENT_ADMIN_MACAROON, "some-macaroon")

    assert on_prem_client._get_macaroon(token_request={}) == "some-macaroon"


def test_get_macaroon_not_in_environment(on_prem_client, monkeypatch):
    """Raise an error if the environment is not setup correctly."""
    monkeypatch.delenv(constants.ENVIRONMENT_ADMIN_MACAROON, raising=False)

    with pytest.raises(errors.SnapcraftError) as raised:
        on_prem_client._get_macaroon(token_request={})

    assert (
        str(raised.value)
        == "'SNAPCRAFT_ADMIN_MACAROON' needs to be setup with a valid macaroon"
    )


def test_get_discharged_macaroon(on_prem_client, mocker):
    """Check that the backend is called correctly."""

    class FakeResponse:
        """Fake response for a discharged macaroon."""

        def json(self):
            return {"macaroon": "discharged-macaroon"}

    request_mock = mocker.patch.object(
        on_prem_client.http_client, "request", return_value=FakeResponse()
    )

    assert (
        on_prem_client._get_discharged_macaroon("root-macaroon")
        == "discharged-macaroon"
    )

    assert request_mock.mock_calls == [
        call(
            "POST",
            "https://fake-store.io/v1/tokens/offline/exchange",
            json={"macaroon": "root-macaroon"},
        )
    ]


def test_get_authorization_header(on_prem_client, mocker):
    """Ensure the correct authorization header is formed for requests"""
    mocker.patch.object(
        on_prem_client._auth,
        "get_credentials",
        return_value="serialized-macaroon-string",
    )
    assert (
        on_prem_client._get_authorization_header()
        == "macaroon serialized-macaroon-string"
    )
