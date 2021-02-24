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
import logging
import os
from typing import Dict

from . import _macaroon_auth, constants, errors
from ._client import Client
from .info import SnapInfo


logger = logging.getLogger(__name__)


class SnapAPI(Client):
    """The Snap API is used to query snaps.

    This is an interface to query that API which is documented
    at http://api.snapcraft.io/docs/.
    """

    def __init__(self, conf):
        """Initialize SnapAPI.

        :param config conf: Configuration details for the client.
        :type config: snapcraft.config.Config
        """
        super().__init__(
            conf, os.environ.get("STORE_API_URL", constants.STORE_API_URL),
        )

    def _get_default_headers(self, api="v2"):
        """Return default headers for CPI requests.

        Tries to build an 'Authorization' header with local credentials
        if they are available.
        Also pin specific branded store if `SNAPCRAFT_UBUNTU_STORE`
        environment is set.
        """
        headers = {}

        with contextlib.suppress(errors.InvalidCredentialsError):
            headers["Authorization"] = _macaroon_auth(self.conf)

        brand_store = os.getenv("SNAPCRAFT_UBUNTU_STORE")
        if brand_store:
            if api == "v2":
                headers["Snap-Device-Store"] = brand_store
            elif api == "v1":
                headers["X-Ubuntu-Store"] = brand_store
            else:
                logger.warning("Incorrect API version passed: {!r}.".format(api))

        return headers

    def get_info(self, snap_name: str, *, arch: str = None) -> SnapInfo:
        """Get the information for the specified snap.

        :param str snap_name: Name of the snap.
        :param str arch: Architecture of the snap (none by default).

        :returns: information for the snap.
        :rtype: SnapInfo
        """
        headers = self._get_default_headers()
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
        url = "/v2/snaps/info/{}".format(snap_name)
        resp = self.get(url, headers=headers, params=params)
        if resp.status_code == 404:
            raise errors.SnapNotFoundError(snap_name=snap_name, arch=arch)
        resp.raise_for_status()

        return SnapInfo(resp.json())

    def get_assertion(
        self, assertion_type: str, snap_id: str
    ) -> Dict[str, Dict[str, str]]:
        """Get the assertion of assertion_type for the specified snap_id.

        :param str assertion_type: The type of assertion.
        :param str snap_id: The ID of the snap.

        :returns: Assertion for the snap.
        """
        headers = self._get_default_headers(api="v1")
        logger.debug("Getting snap-declaration for {}".format(snap_id))
        url = "/api/v1/snaps/assertions/{}/{}/{}".format(
            assertion_type, constants.DEFAULT_SERIES, snap_id
        )
        response = self.get(url, headers=headers)
        if response.status_code != 200:
            raise errors.SnapNotFoundError(snap_id=snap_id)
        return response.json()
