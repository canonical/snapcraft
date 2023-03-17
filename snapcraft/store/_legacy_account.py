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
from craft_cli import emit
from overrides import overrides
from urllib3.util import parse_url
from xdg import BaseDirectory

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
            raise errors.LegacyCredentialsParseError(
                f"Cannot parse config: {parser_error}"
            ) from b64_error

        try:
            parser.read_string(decoded_config_content)
        except configparser.Error as new_parser_error:
            raise errors.LegacyCredentialsParseError(
                f"Cannot parse config: {parser_error}"
            ) from new_parser_error

    return parser


def _deserialize_macaroon(value) -> pymacaroons.Macaroon:
    try:
        return pymacaroons.Macaroon.deserialize(value)
    except:  # noqa LP: #1733004
        raise errors.LegacyCredentialsParseError(  # pylint: disable=raise-missing-from
            "Failed to deserialize macaroon"
        )


def _get_macaroons_from_conf(conf) -> Dict[str, str]:
    """Format a macaroon and its associated discharge.

    :return: A string suitable to use in an Authorization header.
    """
    host = parse_url(os.getenv("UBUNTU_ONE_SSO_URL", constants.UBUNTU_ONE_SSO_URL)).host
    try:
        root_macaroon_raw = conf.get(host, "macaroon")
        unbound_raw = conf.get(host, "unbound_discharge")
    except (configparser.NoOptionError, configparser.NoSectionError) as conf_error:
        raise errors.LegacyCredentialsParseError(str(conf_error)) from conf_error

    return {"r": root_macaroon_raw, "d": unbound_raw}


def get_auth(config_content: str) -> str:
    """Return a legacy authorization header."""
    conf = _load_potentially_base64_config(config_content)
    auth = _get_macaroons_from_conf(conf)

    return base64.b64encode(json.dumps(auth).encode()).decode()


def set_legacy_env() -> None:
    """Set constants.ENVIRONMENT_STORE_CREDENTIALS to a valid value.

    Transform the configparser based environment into a value useful
    for craft-store.
    """
    if LegacyUbuntuOne.env_has_legacy_credentials():
        emit.trace(
            f"Found legacy credentials exported on {constants.ENVIRONMENT_STORE_CREDENTIALS!r}"
        )
        auth = get_auth(
            config_content=os.getenv(constants.ENVIRONMENT_STORE_CREDENTIALS)  # type: ignore
        )
        os.environ[constants.ENVIRONMENT_STORE_CREDENTIALS] = auth
    elif LegacyUbuntuOne.has_legacy_credentials():
        emit.trace(
            f"Found legacy credentials stored in {LegacyUbuntuOne.CONFIG_PATH!r}"
        )
        config_content = LegacyUbuntuOne.CONFIG_PATH.read_text()
        auth = get_auth(config_content=config_content)
        os.environ[constants.ENVIRONMENT_STORE_CREDENTIALS] = auth


class LegacyUbuntuOne(craft_store.UbuntuOneStoreClient):
    """Legacy client to easily transition existing CI users."""

    CONFIG_PATH = Path(BaseDirectory.xdg_config_home) / "snapcraft/snapcraft.cfg"

    @classmethod
    def env_has_legacy_credentials(cls) -> bool:
        """Return True if legacy credentials are exported in the environment."""
        credentials = os.getenv(constants.ENVIRONMENT_STORE_CREDENTIALS)
        if credentials is None:
            return False

        try:
            get_auth(credentials)
            emit.trace(
                f"Found legacy credentials exported on {constants.ENVIRONMENT_STORE_CREDENTIALS}"
            )
        except errors.LegacyCredentialsParseError:
            return False

        return True

    @classmethod
    def has_legacy_credentials(cls) -> bool:
        """Return True if legacy credentials are stored."""
        if cls.CONFIG_PATH.exists():
            emit.trace(f"Found legacy credentials stored in {cls.CONFIG_PATH}")
            return True

        return False

    @classmethod
    def store_credentials(cls, config_content) -> None:
        """Store legacy credentials."""
        # Check to see if the content is valid.
        get_auth(config_content)
        cls.CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
        cls.CONFIG_PATH.write_text(config_content)

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
        # Adapt to the JSON format if the environment has configparser based credentials.
        if self.env_has_legacy_credentials():
            auth = get_auth(
                config_content=os.getenv(constants.ENVIRONMENT_STORE_CREDENTIALS)  # type: ignore
            )
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

        # Now that an Auth instance exists, if configparser based credentials
        # exist load them and set them.
        if self.has_legacy_credentials():
            config_content = LegacyUbuntuOne.CONFIG_PATH.read_text()
            auth = self._auth.decode_credentials(
                get_auth(config_content=config_content)
            )
            self._auth.set_credentials(auth)

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
        raise errors.SnapcraftError(
            "Cannot login with existing legacy credentials in use",
            resolution="Run 'snapcraft logout' first to clear them",
        )

    @overrides
    def logout(self) -> None:
        """Logout by removing legacy credentials."""
        emit.trace(f"Clearing legacy credentials from {self.CONFIG_PATH!r}")
        self.CONFIG_PATH.unlink(missing_ok=True)
