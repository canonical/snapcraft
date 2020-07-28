# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2020 Canonical Ltd
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

import contextlib
import os
from typing import Dict

from ._client import Client
from .info import SnapInfo

from . import errors
from . import logger, _macaroon_auth
from . import constants


class SnapIndexClient(Client):
    """The Click Package Index knows everything about existing snaps.
    https://wiki.ubuntu.com/AppStore/Interfaces/ClickPackageIndex is the
    canonical reference.
    """

    def __init__(self, conf):
        """Initialize the SnapIndexClient object.

        :param config conf: Configuration details for the client.
        :type config: snapcraft.config.Config
        """
        super().__init__(
            conf,
            os.environ.get(
                "UBUNTU_STORE_SEARCH_ROOT_URL", constants.UBUNTU_STORE_SEARCH_ROOT_URL
            ),
        )

    def get_default_headers(self, api="v2"):
        """Return default headers for CPI requests.
        Tries to build an 'Authorization' header with local credentials
        if they are available.
        Also pin specific branded store if `SNAPCRAFT_UBUNTU_STORE`
        environment is set.
        """
        headers = {}

        with contextlib.suppress(errors.InvalidCredentialsError):
            headers["Authorization"] = _macaroon_auth(self.conf)

        branded_store = os.getenv("SNAPCRAFT_UBUNTU_STORE")
        if branded_store:
            if api == "v2":
                headers["Snap-Device-Store"] = branded_store
            elif api == "v1":
                headers["X-Ubuntu-Store"] = branded_store
            else:
                logger.warning("Incorrect API version passed: {!r}.".format(api))

        return headers

    def get_info(self, snap_name: str, *, arch: str = None) -> SnapInfo:
        """Get the information for the specified snap.

        :param str snap_name: Name of the snap.
        :param str arch: Architecture of the snap (none by default).

        :return information for the snap.
        :rtype: SnapInfo
        """
        headers = self.get_default_headers()
        headers.update(
            {
                "Accept": "application/json",
                "Snap-Device-Series": constants.DEFAULT_SERIES,
            }
        )

        params = dict()
        params[
            "fields"
        ] = "channel-map,snap-id,name,publisher,confinement,revision,download"
        if arch is not None:
            params["architecture"] = arch
        logger.debug("Getting information for {}".format(snap_name))
        url = "v2/snaps/info/{}".format(snap_name)
        resp = self.get(url, headers=headers, params=params)
        if resp.status_code == 404:
            raise errors.SnapNotFoundError(snap_name=snap_name, arch=arch)
        resp.raise_for_status()

        return SnapInfo(resp.json())

    def get_assertion(
        self, assertion_type: str, snap_id: str
    ) -> Dict[str, Dict[str, str]]:
        """Get the assertion for the specified snap.

        :param str assertion_type: The type of the assertion.
        :param str snap_id: The ID of the snap.

        :return Assertion for the snap.
        """
        headers = self.get_default_headers(api="v1")
        logger.debug("Getting snap-declaration for {}".format(snap_id))
        url = "/api/v1/snaps/assertions/{}/{}/{}".format(
            assertion_type, constants.DEFAULT_SERIES, snap_id
        )
        response = self.get(url, headers=headers)
        if response.status_code != 200:
            raise errors.SnapNotFoundError(snap_id=snap_id)
        return response.json()

    def get(self, url, headers=None, params=None, stream=False):
        """Perform a GET request with the given arguments.

        :param str url: URL to send the request.
        :param dict headers: Headers to be sent along with the request.
        :param dict params: Query parameters to be sent along with
        the request.
        :param bool stream: Determines if the request shouldn't be
        automatically closed (true by default).

        :return Response of the request.
        """
        if headers is None:
            headers = self.get_default_headers()
        response = self.request(
            "GET", url, stream=stream, headers=headers, params=params
        )
        return response
