#!/usr/bin/env python3

import logging
import json
import os
import pathlib
from typing import Optional, TextIO
from urllib.parse import urljoin, urlparse

import pymacaroons
import requests
from requests.adapters import HTTPAdapter
from requests.exceptions import ConnectionError, RetryError
from requests.packages.urllib3.util.retry import Retry
from simplejson.scanner import JSONDecodeError
from xdg import BaseDirectory

from . import _agent, constants, errors
from ._config.base_config import Config

logger = logging.getLogger(__name__)

# Set urllib3's logger to only emit errors, not warnings. Otherwise even
# retries are printed, and they're nasty.
logging.getLogger(requests.packages.urllib3.__package__).setLevel(logging.ERROR)


class Client:
    """Generic Client to talk to the *Store."""

    def __init__(self, *, user_agent: str = _agent.get_user_agent()) -> None:
        self.session = requests.Session()
        self._user_agent = user_agent

        # Setup max retries for all store URLs and the CDN
        retries = Retry(
            total=int(os.environ.get("STORE_RETRIES", 5)),
            backoff_factor=int(os.environ.get("STORE_BACKOFF", 2)),
            status_forcelist=[104, 500, 502, 503, 504],
        )
        self.session.mount("http://", HTTPAdapter(max_retries=retries))
        self.session.mount("https://", HTTPAdapter(max_retries=retries))

    def request(
        self, method, url, params=None, headers=None, **kwargs
    ) -> requests.Response:
        """Send a request to url relative to the root url.

        :param str method: Method used for the request.
        :param str url: URL to request with method.
        :param list params: Query parameters to be sent along with the request.
        :param list headers: Headers to be sent along with the request.

        :return Response of the request.
        """
        if headers:
            headers["User-Agent"] = self._user_agent
        else:
            headers = {"User-Agent": self._user_agent}

        debug_headers = headers.copy()
        if "Authorization" in debug_headers:
            debug_headers["Authorization"] = "<macaroon>"
        logger.debug(
            "Calling {} with params {} and headers {}".format(
                url, params, debug_headers
            )
        )
        try:
            response = self.session.request(
                method, url, headers=headers, params=params, **kwargs
            )
        except (ConnectionError, RetryError) as e:
            raise errors.StoreNetworkError(e) from e

        # Handle 5XX responses generically right here, so the callers don't
        # need to worry about it.
        if response.status_code >= 500:
            raise errors.StoreServerError(response)

        return response


def _deserialize_macaroon(value):
    try:
        return pymacaroons.Macaroon.deserialize(value)
    except:  # noqa LP: #1733004
        raise errors.InvalidCredentialsError("Failed to deserialize macaroon")


def _macaroon_auth(conf):
    """Format a macaroon and its associated discharge.

    :return: A string suitable to use in an Authorization header.

    """
    root_macaroon_raw = conf.get("macaroon")
    if root_macaroon_raw is None:
        raise errors.InvalidCredentialsError("Root macaroon not in the config file")
    unbound_raw = conf.get("unbound_discharge")
    if unbound_raw is None:
        raise errors.InvalidCredentialsError("Unbound discharge not in the config file")

    root_macaroon = _deserialize_macaroon(root_macaroon_raw)
    unbound = _deserialize_macaroon(unbound_raw)
    bound = root_macaroon.prepare_for_request(unbound)
    discharge_macaroon_raw = bound.serialize()
    auth = "Macaroon root={}, discharge={}".format(
        root_macaroon_raw, discharge_macaroon_raw
    )

    return auth


class UbuntuOneSSOConfig(Config):
    """Hold configuration options in sections.

    There can be two sections for the sso related credentials: production and
    staging. This is governed by the UBUNTU_ONE_SSO_URL environment
    variable. Other sections are ignored but preserved.

    """

    def _get_section_name(self) -> str:
        url = os.getenv("UBUNTU_ONE_SSO_URL", constants.UBUNTU_ONE_SSO_URL)
        return urlparse(url).netloc

    def _get_config_path(self) -> pathlib.Path:
        return (
            pathlib.Path(BaseDirectory.save_config_path("snapcraft")) / "snapcraft.cfg"
        )


class UbuntuOneAuthClient(Client):
    """Store Client using Ubuntu One SSO provided macaroons."""

    @staticmethod
    def _is_needs_refresh_response(response):
        return (
            response.status_code == requests.codes.unauthorized
            and response.headers.get("WWW-Authenticate") == "Macaroon needs_refresh=1"
        )

    def __init__(self, *, user_agent: str = _agent.get_user_agent()) -> None:
        super().__init__(user_agent=user_agent)

        self._conf = UbuntuOneSSOConfig()
        self.auth_url = os.environ.get(
            "UBUNTU_ONE_SSO_URL", constants.UBUNTU_ONE_SSO_URL
        )

        try:
            self.auth: Optional[str] = _macaroon_auth(self._conf)
        except errors.InvalidCredentialsError:
            self.auth = None

    def _extract_caveat_id(self, root_macaroon):
        macaroon = pymacaroons.Macaroon.deserialize(root_macaroon)
        # macaroons are all bytes, never strings
        sso_host = urlparse(self.auth_url).netloc
        for caveat in macaroon.caveats:
            if caveat.location == sso_host:
                return caveat.caveat_id
        else:
            raise errors.InvalidCredentialsError("Invalid root macaroon")

    def login(
        self,
        *,
        email: Optional[str] = None,
        password: Optional[str] = None,
        macaroon: Optional[str] = None,
        otp: Optional[str] = None,
        config_fd: TextIO = None,
        save: bool = True,
    ) -> None:
        if config_fd is not None:
            self._conf.load(config_fd=config_fd)
        # Verbose to keep static checks happy.
        elif email is not None and password is not None and macaroon is not None:
            # Ask the store for the needed capabilities to be associated with
            # the macaroon.
            caveat_id = self._extract_caveat_id(macaroon)
            unbound_discharge = self._discharge_token(email, password, otp, caveat_id)
            # Clear any old data before setting.
            self._conf.clear()
            # The macaroon has been discharged, save it in the config
            self._conf.set("macaroon", macaroon)
            self._conf.set("unbound_discharge", unbound_discharge)
            self._conf.set("email", email)
            # Set auth and headers.
            self.auth = _macaroon_auth(self._conf)
        else:
            raise RuntimeError("Logic Error")

        if save:
            self._conf.save()

    def export_login(self, *, config_fd: TextIO, encode: bool = False) -> None:
        self._conf.save(config_fd=config_fd, encode=encode)

    def logout(self) -> None:
        self._conf.clear()
        self._conf.save()

    def _discharge_token(
        self, email: str, password: str, otp: Optional[str], caveat_id
    ) -> str:
        data = dict(email=email, password=password, caveat_id=caveat_id)
        if otp:
            data["otp"] = otp

        url = urljoin(self.auth_url, "/api/v2/tokens/discharge")

        response = self.request(
            "POST",
            url,
            data=json.dumps(data),
            headers={"Content-Type": "application/json", "Accept": "application/json"},
        )

        if response.ok:
            return response.json()["discharge_macaroon"]

        try:
            response_json = response.json()
        except JSONDecodeError:
            response_json = dict()

        if response.status_code == requests.codes.unauthorized and any(
            error.get("code") == "twofactor-required"
            for error in response_json.get("error_list", [])
        ):
            raise errors.StoreTwoFactorAuthenticationRequired()
        else:
            raise errors.StoreAuthenticationError(
                "Failed to get unbound discharge", response
            )

    def _refresh_token(self, unbound_discharge):
        data = {"discharge_macaroon": unbound_discharge}
        url = urljoin(self.auth_url, "/api/v2/tokens/refresh")
        response = self.request(
            "POST",
            url,
            json=data,
            headers={"Content-Type": "application/json", "Accept": "application/json"},
        )
        if response.ok:
            return response.json()["discharge_macaroon"]
        else:
            raise errors.StoreAuthenticationError(
                "Failed to refresh unbound discharge", response
            )

    def request(
        self, method, url, params=None, headers=None, auth_header=True, **kwargs
    ) -> requests.Response:
        if headers and auth_header:
            headers["Authorization"] = self.auth
        elif auth_header:
            headers = {"Authorization": self.auth}

        response = super().request(
            method, url, params=params, headers=headers, **kwargs
        )

        if self._is_needs_refresh_response(response):
            unbound_discharge = self._refresh_token(self._conf.get("unbound_discharge"))
            self._conf.set("unbound_discharge", unbound_discharge)
            self._conf.save()
            self.auth = _macaroon_auth(self._conf)
            headers["Authorization"] = self.auth

            response = super().request(
                method, url, params=params, headers=headers, **kwargs
            )

        return response
