# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2020 Canonical Ltd
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

import requests

from ._client import Client

from . import constants
from . import errors
from . import _macaroon_auth
from .v2 import channel_map


class SnapV2Client(Client):
    """The software center agent deals with managing snaps."""

    def __init__(self, conf):
        super().__init__(
            conf,
            os.environ.get(
                "SNAP_STORE_DASHBOARD_ROOT_URL", constants.SNAP_STORE_DASHBOARD_ROOT_URL
            )
            + "/api/v2/snaps/",
        )

    @staticmethod
    def _is_needs_refresh_response(response):
        return (
            response.status_code == requests.codes.unauthorized
            and response.headers.get("WWW-Authenticate") == "Macaroon needs_refresh=1"
        )

    def request(self, *args, **kwargs):
        response = super().request(*args, **kwargs)
        if self._is_needs_refresh_response(response):
            raise errors.StoreMacaroonNeedsRefreshError()
        return response

    def get_snap_channel_map(self, *, snap_name: str) -> channel_map.ChannelMap:
        url = snap_name + "/channel-map"
        auth = _macaroon_auth(self.conf)
        response = self.get(
            url,
            headers={
                "Authorization": auth,
                "Content-Type": "application/json",
                "Accept": "application/json",
            },
        )

        if not response.ok:
            raise errors.StoreSnapChannelMapError(snap_name=snap_name)

        return channel_map.ChannelMap.unmarshal(response.json())
