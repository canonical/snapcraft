# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2020-2022 Canonical Ltd.
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

"""Personal Package Archive helpers."""

from typing import Tuple

import lazr.restfulclient.errors
from craft_cli import emit
from launchpadlib.launchpad import Launchpad

from . import errors


def split_ppa_parts(*, ppa: str) -> Tuple[str, str]:
    """Obtain user and repository components from a PPA line."""
    ppa_split = ppa.split("/")
    if len(ppa_split) != 2:
        raise errors.AptPPAInstallError(ppa, "invalid PPA format")
    return ppa_split[0], ppa_split[1]


def get_launchpad_ppa_key_id(*, ppa: str) -> str:
    """Query Launchpad for PPA's key ID."""
    owner, name = split_ppa_parts(ppa=ppa)
    launchpad = Launchpad.login_anonymously("snapcraft", "production")
    launchpad_url = f"~{owner}/+archive/{name}"

    emit.debug(f"Loading launchpad url: {launchpad_url}")
    try:
        key_id = launchpad.load(launchpad_url).signing_key_fingerprint
    except lazr.restfulclient.errors.NotFound as error:
        raise errors.AptPPAInstallError(ppa, "not found on launchpad") from error

    emit.debug(f"Retrieved launchpad PPA key ID: {key_id}")

    return key_id
