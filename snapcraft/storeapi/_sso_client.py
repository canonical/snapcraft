import json
import os

import requests
from simplejson.scanner import JSONDecodeError

from . import constants, errors
from ._client import Client


class SSOClient(Client):
    """The Single Sign On server deals with authentication.
    It is used directly or indirectly by other servers.
    """

    def __init__(self, conf):
        super().__init__(
            conf, os.environ.get("UBUNTU_ONE_SSO_URL", constants.UBUNTU_ONE_SSO_URL),
        )

    def get_unbound_discharge(self, email, password, one_time_password, caveat_id):
        data = dict(email=email, password=password, caveat_id=caveat_id)
        if one_time_password:
            data["otp"] = one_time_password
        response = self.post(
            "/api/v2/tokens/discharge",
            data=json.dumps(data),
            headers={"Content-Type": "application/json", "Accept": "application/json"},
        )
        try:
            response_json = response.json()
        except JSONDecodeError:
            response_json = {}
        if response.ok:
            return response_json["discharge_macaroon"]
        else:
            if response.status_code == requests.codes.unauthorized and any(
                error.get("code") == "twofactor-required"
                for error in response_json.get("error_list", [])
            ):
                raise errors.StoreTwoFactorAuthenticationRequired()
            else:
                raise errors.StoreAuthenticationError(
                    "Failed to get unbound discharge", response
                )

    def refresh_unbound_discharge(self, unbound_discharge):
        data = {"discharge_macaroon": unbound_discharge}
        response = self.post(
            "/api/v2/tokens/refresh",
            data=json.dumps(data),
            headers={"Content-Type": "application/json", "Accept": "application/json"},
        )
        if response.ok:
            return response.json()["discharge_macaroon"]
        else:
            raise errors.StoreAuthenticationError(
                "Failed to refresh unbound discharge", response
            )
