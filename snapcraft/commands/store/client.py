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

"""Snapcraft Store Client with CLI hooks."""

import os
import platform
from datetime import timedelta
from typing import Any, Dict, Optional, Sequence, Tuple

import craft_store
from craft_cli import emit

from snapcraft import __version__, utils

from . import constants

_TESTING_ENV_PREFIXES = ["TRAVIS", "AUTOPKGTEST_TMP"]


def build_user_agent(
    version=__version__, os_platform: utils.OSPlatform = utils.get_os_platform()
):
    """Build Snapcraft's user agent."""
    if any(
        key.startswith(prefix) for prefix in _TESTING_ENV_PREFIXES for key in os.environ
    ):
        testing = " (testing) "
    else:
        testing = " "
    return f"snapcraft/{version}{testing}{os_platform!s}"


def use_candid() -> bool:
    """Return True if using candid as the auth backend."""
    return os.getenv(constants.ENVIRONMENT_STORE_AUTH) == "candid"


def get_store_url() -> str:
    """Return the Snap Store url considering the environment."""
    return os.getenv("STORE_DASHBOARD_URL", constants.STORE_URL)


def get_store_upload_url() -> str:
    """Return the Snap Store Upload url considering the environment."""
    return os.getenv("STORE_UPLOAD_URL", constants.STORE_UPLOAD_URL)


def get_store_login_url() -> str:
    """Return the Ubuntu Login url considering the environment.

    This is only useful when using Ubuntu One SSO.
    """
    return os.getenv("UBUNTU_ONE_SSO_URL", constants.UBUNTU_ONE_SSO_URL)


def _prompt_login() -> Tuple[str, str]:
    emit.message(
        "Enter your Ubuntu One e-mail address and password.", intermediate=True
    )
    emit.message(
        "If you do not have an Ubuntu One account, you can create one "
        "at https://snapcraft.io/account",
        intermediate=True,
    )
    email = utils.prompt("Email: ")
    password = utils.prompt("Password: ", hide=True)

    return (email, password)


def _get_hostname(hostname: Optional[str] = platform.node()) -> str:
    """Return the computer's network name or UNNKOWN if it cannot be determined."""
    if not hostname:
        hostname = "UNKNOWN"
    return hostname


def get_client(ephemeral: bool) -> craft_store.BaseClient:
    """Store Client factory."""
    store_url = get_store_url()
    store_upload_url = get_store_upload_url()
    user_agent = build_user_agent()

    if use_candid() is True:
        client: craft_store.BaseClient = craft_store.StoreClient(
            base_url=store_url,
            storage_base_url=store_upload_url,
            application_name="snapcraft",
            user_agent=user_agent,
            endpoints=craft_store.endpoints.SNAP_STORE,
            environment_auth=constants.ENVIRONMENT_STORE_CREDENTIALS,
            ephemeral=ephemeral,
        )
    else:
        client = craft_store.UbuntuOneStoreClient(
            base_url=store_url,
            storage_base_url=store_upload_url,
            auth_url=get_store_login_url(),
            application_name="snapcraft",
            user_agent=user_agent,
            endpoints=craft_store.endpoints.U1_SNAP_STORE,
            environment_auth=constants.ENVIRONMENT_STORE_CREDENTIALS,
            ephemeral=ephemeral,
        )

    return client


class StoreClientCLI:
    """A BaseClient implementation considering command line prompts."""

    def __init__(self, ephemeral=False):
        self.store_client = get_client(ephemeral=ephemeral)

    def login(
        self,
        *,
        ttl: int = int(timedelta(days=365).total_seconds()),
        acls: Optional[Sequence[str]] = None,
        packages: Optional[Sequence[str]] = None,
        channels: Optional[Sequence[str]] = None,
    ) -> str:
        """Login to the Snap Store and prompt if required."""
        kwargs: Dict[str, Any] = {}
        if use_candid() is False:
            kwargs["email"], kwargs["password"] = _prompt_login()

        if packages is None:
            packages = []
        _packages = [
            craft_store.endpoints.Package(package_name=p, package_type="snap")
            for p in packages
        ]
        if acls is None:
            acls = [
                "package_access",
                "package_manage",
                "package_metrics",
                "package_push",
                "package_register",
                "package_release",
                "package_update",
            ]

        description = f"snapcraft@{_get_hostname()}"

        try:
            credentials = self.store_client.login(
                ttl=ttl,
                permissions=acls,
                channels=channels,
                packages=_packages,
                description=description,
                **kwargs,
            )
        except craft_store.errors.StoreServerError as store_error:
            if "twofactor-required" not in store_error.error_list:
                raise
            kwargs["otp"] = utils.prompt("Second-factor auth: ")

            credentials = self.store_client.login(
                ttl=ttl,
                permissions=acls,
                channels=channels,
                packages=_packages,
                description=description,
                **kwargs,
            )

        return credentials
