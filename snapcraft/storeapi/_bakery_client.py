import base64
import json
import os
from typing import Optional, TextIO

import requests
import macaroonbakery._utils as utils
from macaroonbakery import bakery, httpbakery

from ._ubuntu_sso_client import Client
from . import _agent


class BakeryClient(Client):
    def __init__(self, *, user_agent: str = _agent.get_user_agent()) -> None:
        super().__init__(user_agent=user_agent)

        self.bakery_client = httpbakery.Client()
        self.auth = None

    def login(
        self, *, macaroon: Optional[str] = None, config_fd: Optional[TextIO] = None
    ) -> None:
        if config_fd is not None:
            raise RuntimeError("Not supported")
        if macaroon is None:
            raise RuntimeError("Logic Error")

        bakery_macaroon = bakery.Macaroon.from_dict(json.loads(macaroon))
        discharges = bakery.discharge_all(
            bakery_macaroon, self.bakery_client.acquire_discharge
        )

        # serialize macaroons the bakery-way
        discharged_macaroons = (
            "[" + ",".join(map(utils.macaroon_to_json_string, discharges)) + "]"
        )

        self.auth = base64.urlsafe_b64encode(
            utils.to_bytes(discharged_macaroons)
        ).decode("ascii")
        self.macaroon = macaroon

    def request(
        self, method, url, params=None, headers=None, auth_header=True, **kwargs
    ) -> requests.Response:
        if headers and auth_header:
            headers["Macaroons"] = self.auth
        elif auth_header:
            headers = {"Macaroons": self.auth}

        response = super().request(
            method, url, params=params, headers=headers, **kwargs
        )

        if not response.ok and response.status_code == 401:
            self.login(macaroon=self.macaroon)

            response = super().request(
                method, url, params=params, headers=headers, **kwargs
            )

        return response

    def export_login(self, *, config_fd: TextIO, encode: bool):
        pass

    def logout(self) -> None:
        pass
