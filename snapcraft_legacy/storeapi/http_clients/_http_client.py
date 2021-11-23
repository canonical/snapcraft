# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2021 Canonical Ltd
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

import os
import logging

import requests
from requests.adapters import HTTPAdapter
from requests.exceptions import ConnectionError, RetryError
from requests.packages.urllib3.util.retry import Retry

from . import agent, errors


# Set urllib3's logger to only emit errors, not warnings. Otherwise even
# retries are printed, and they're nasty.
logging.getLogger(requests.packages.urllib3.__package__).setLevel(logging.ERROR)
logger = logging.getLogger(__name__)


class Client:
    """Generic Client to talk to the *Store."""

    def __init__(self, *, user_agent: str = agent.get_user_agent()) -> None:
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
        if debug_headers.get("Authorization"):
            debug_headers["Authorization"] = "<macaroon>"
        if debug_headers.get("Macaroons"):
            debug_headers["Macaroons"] = "<macaroon>"
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
