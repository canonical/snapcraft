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

"""On premises Store Client implementation."""

import os
from typing import Any, Dict, Final

import craft_store.creds
from craft_store import BaseClient, endpoints, models
from overrides import overrides

from snapcraft import errors

from . import constants

ON_PREM_ENDPOINTS: Final = endpoints.Endpoints(
    namespace="snap",
    whoami="/v1/tokens/whoami",
    tokens="",  # noqa: S106
    tokens_exchange="/v1/tokens/offline/exchange",  # noqa: S106
    valid_package_types=["snap"],
    list_releases_model=models.charm_list_releases_model.ListReleasesModel,
)


class OnPremClient(BaseClient):
    """On Premises Snapcraft Store Client."""

    @overrides
    def _get_macaroon(self, token_request: Dict[str, Any]) -> str:
        macaroon_env = os.getenv(constants.ENVIRONMENT_ADMIN_MACAROON)
        if macaroon_env is None:
            raise errors.SnapcraftError(
                f"{constants.ENVIRONMENT_ADMIN_MACAROON!r} needs to be setup with a valid macaroon"
            )
        return macaroon_env

    @overrides
    def _get_discharged_macaroon(self, root_macaroon: str, **kwargs) -> str:
        response = self.http_client.request(
            "POST",
            self._base_url + self._endpoints.tokens_exchange,
            json={"macaroon": root_macaroon},
        )

        return craft_store.creds.marshal_candid_credentials(response.json()["macaroon"])

    @overrides
    def _get_authorization_header(self) -> str:
        auth = craft_store.creds.unmarshal_candid_credentials(
            self._auth.get_credentials()
        )
        return f"macaroon {auth}"
