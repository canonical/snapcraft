# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2017 Canonical Ltd
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

import logging

import pymacaroons

from . import errors

logger = logging.getLogger(__name__)


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


def _deserialize_macaroon(value):
    try:
        return pymacaroons.Macaroon.deserialize(value)
    except:  # noqa LP: #1733004
        raise errors.InvalidCredentialsError("Failed to deserialize macaroon")


from ._store_client import StoreClient  # noqa
