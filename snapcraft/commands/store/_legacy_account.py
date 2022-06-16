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

"""Backwards compatibility for login --with."""

import base64
import configparser
import json
import os
from pathlib import Path
from typing import Dict, Optional, Sequence

import craft_store
import pymacaroons
import xdg.BaseDirectory
from overrides import overrides
from urllib3.util import parse_url

from snapcraft import errors

from . import constants


def _load_potentially_base64_config(config_content: str) -> configparser.ConfigParser:
    parser = configparser.ConfigParser()
    try:
        parser.read_string(config_content)
    except configparser.Error as parser_error:
        # The config may be base64-encoded, try decoding it
        try:
            decoded_config_content = base64.b64decode(config_content).decode()
        except base64.binascii.Error as b64_error:  # type: ignore
            # It wasn't base64, so use the original error
            raise errors.SnapcraftError(
                f"Cannot parse config: {parser_error}"
            ) from b64_error

        try:
            parser.read_string(decoded_config_content)
        except configparser.Error as new_parser_error:
            raise errors.SnapcraftError(
                f"Cannot parse config: {parser_error}"
            ) from new_parser_error

    return parser


def _deserialize_macaroon(value) -> pymacaroons.Macaroon:
    try:
        return pymacaroons.Macaroon.deserialize(value)
    except:  # noqa LP: #1733004
        raise errors.SnapcraftError(  # pylint: disable=raise-missing-from
            "Failed to deserialize macaroon"
        )


def _get_macaroons_from_conf(conf) -> Dict[str, str]:
    """Format a macaroon and its associated discharge.

    :return: A string suitable to use in an Authorization header.
    """
    host = parse_url(constants.UBUNTU_ONE_SSO_URL).host
    try:
        root_macaroon_raw = conf.get(host, "macaroon")
        unbound_raw = conf.get(host, "unbound_discharge")
    except configparser.NoOptionError as conf_error:
        raise errors.SnapcraftError(str(conf_error)) from conf_error

    return {"r": root_macaroon_raw, "d": unbound_raw}


def get_auth(config_content: str) -> str:
    """Return a legacy authorization header."""
    conf = _load_potentially_base64_config(config_content)
    auth = _get_macaroons_from_conf(conf)

    return base64.b64encode(json.dumps(auth).encode()).decode()


class LegacyUbuntuOne(craft_store.UbuntuOneStoreClient):
    """Legacy client to easily transition existing CI users."""

    _CONFIG_PATH = Path(xdg.BaseDirectory.xdg_config_home) / "snapcraft/snapcraft.cfg"

    @classmethod
    def has_legacy_credentials(cls) -> bool:
        """Return True if legacy credentials are stored."""
        return cls._CONFIG_PATH.exists()

    @classmethod
    def store_credentials(cls, config_content) -> None:
        """Store legacy credentials."""
        cls._CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
        cls._CONFIG_PATH.write_text(config_content)

    @overrides
    def __init__(
        self,
        *,
        base_url: str,
        storage_base_url: str,
        auth_url: str,
        endpoints: craft_store.endpoints.Endpoints,  # pylint: disable=W0621
        application_name: str,
        user_agent: str,
        environment_auth: Optional[str] = None,
        ephemeral: bool = False,
    ) -> None:
        if self.has_legacy_credentials():
            config_content = self._CONFIG_PATH.read_text()
            auth = get_auth(config_content=config_content)
            os.environ[constants.ENVIRONMENT_STORE_CREDENTIALS] = auth

        super().__init__(
            base_url=base_url,
            storage_base_url=storage_base_url,
            auth_url=auth_url,
            application_name=application_name,
            user_agent=user_agent,
            endpoints=endpoints,
            environment_auth=constants.ENVIRONMENT_STORE_CREDENTIALS,
            ephemeral=True,
        )

    @overrides
    def login(
        self,
        *,
        permissions: Sequence[str],
        description: str,
        ttl: int,
        packages: Optional[Sequence[craft_store.endpoints.Package]] = None,
        channels: Optional[Sequence[str]] = None,
        **kwargs,
    ) -> str:
        raise NotImplementedError("Cannot login with legacy")

    @overrides
    def logout(self) -> None:
        """Logout by removing legacy credentials."""
        self._CONFIG_PATH.unlink(missing_ok=True)
