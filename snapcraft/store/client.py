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
import time
from datetime import timedelta
from typing import Any, Dict, List, Optional, Sequence, Tuple, cast

import craft_store
import requests
from craft_cli import emit
from overrides import overrides

from snapcraft import __version__, errors, utils
from snapcraft_legacy.storeapi.v2.releases import Releases as Revisions

from . import channel_map, constants
from ._legacy_account import LegacyUbuntuOne
from .onprem_client import ON_PREM_ENDPOINTS, OnPremClient

_TESTING_ENV_PREFIXES = ["TRAVIS", "AUTOPKGTEST_TMP"]

_POLL_DELAY = 1
_HUMAN_STATUS = {
    "being_processed": "processing",
    "ready_to_release": "ready to release!",
    "need_manual_review": "will need manual review",
    "processing_upload_delta_error": "error while processing delta",
    "processing_error": "error while processing",
}


def build_user_agent(
    version=__version__,
    os_platform: utils.OSPlatform = utils.get_os_platform(),  # noqa: B008
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


def is_onprem() -> bool:
    """Return True if using onprem as the auth backend."""
    return os.getenv(constants.ENVIRONMENT_STORE_AUTH) == "onprem"


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
    emit.message("Enter your Ubuntu One e-mail address and password.")
    emit.message(
        "If you do not have an Ubuntu One account, you can create one "
        "at https://snapcraft.io/account",
    )
    email = utils.prompt("Email: ")
    password = utils.prompt("Password: ", hide=True)

    return (email, password)


def _get_hostname(
    hostname: Optional[
        str
    ] = platform.node(),  # noqa: B008 Function call in arg defaults
) -> str:
    """Return the computer's network name or UNNKOWN if it cannot be determined."""
    if not hostname:
        hostname = "UNKNOWN"
    return hostname


def get_client(ephemeral: bool) -> craft_store.BaseClient:
    """Store Client factory."""
    store_url = get_store_url()
    store_upload_url = get_store_upload_url()
    user_agent = build_user_agent()

    # Legacy will be used when:
    # 1. the current environment was set to use the legacy credentials
    # 2. login --with or existing legacy credentials are found
    # And will not be used if ephemeral is set (export-login)
    use_legacy = not ephemeral and (
        LegacyUbuntuOne.has_legacy_credentials()
        or LegacyUbuntuOne.env_has_legacy_credentials()
    )

    if is_onprem():
        client: craft_store.BaseClient = OnPremClient(
            base_url=store_url,
            storage_base_url=store_upload_url,
            application_name="snapcraft",
            user_agent=user_agent,
            endpoints=ON_PREM_ENDPOINTS,
            environment_auth=constants.ENVIRONMENT_STORE_CREDENTIALS,
            ephemeral=ephemeral,
        )
    elif use_legacy:
        client = LegacyUbuntuOne(
            base_url=store_url,
            storage_base_url=store_upload_url,
            auth_url=get_store_login_url(),
            application_name="snapcraft",
            user_agent=user_agent,
            endpoints=craft_store.endpoints.U1_SNAP_STORE,
            environment_auth=constants.ENVIRONMENT_STORE_CREDENTIALS,
            ephemeral=ephemeral,
        )
    elif use_candid() is True:
        client = craft_store.StoreClient(
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


class LegacyStoreClientCLI:
    """A BaseClient implementation considering command line prompts."""

    def __init__(self, ephemeral=False):
        self.store_client = get_client(ephemeral=ephemeral)
        self._base_url = get_store_url()

    def login(
        self,
        *,
        ttl: int = int(timedelta(days=365).total_seconds()),  # noqa: B008
        acls: Optional[Sequence[str]] = None,
        packages: Optional[Sequence[str]] = None,
        channels: Optional[Sequence[str]] = None,
        **kwargs,
    ) -> str:
        """Log in to the Snap Store and prompt if required."""
        if os.getenv(constants.ENVIRONMENT_STORE_CREDENTIALS):
            raise errors.SnapcraftError(
                f"Cannot login with {constants.ENVIRONMENT_STORE_CREDENTIALS!r} set.",
                resolution=f"Unset {constants.ENVIRONMENT_STORE_CREDENTIALS!r} and try again.",
            )

        if not is_onprem() and use_candid() is False:
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

    def request(self, *args, **kwargs) -> requests.Response:
        """Request using the BaseClient and wrap responses that require action.

        Actionable items are those that could prompt a login or registration.
        """
        try:
            return self.store_client.request(*args, **kwargs)
        except craft_store.errors.StoreServerError as store_error:
            if (
                store_error.response.status_code
                == requests.codes.unauthorized  # pylint: disable=no-member
            ):
                if os.getenv(constants.ENVIRONMENT_STORE_CREDENTIALS):
                    raise errors.StoreCredentialsUnauthorizedError(
                        "Exported credentials are no longer valid for the Snap Store.",
                        resolution=(
                            "Run export-login and update "
                            f"{constants.ENVIRONMENT_STORE_CREDENTIALS}."
                        ),
                    ) from store_error
                self.store_client.logout()
                # Make it a manual process to login again as these older credentials
                # might be part of some CI/CD workflow.
                if isinstance(self.store_client, LegacyUbuntuOne):
                    raise errors.StoreCredentialsUnauthorizedError(
                        "Credentials are no longer valid for the Snap Store.",
                        resolution=(
                            "Run snapcraft login or export-login to obtain "
                            "new credentials."
                        ),
                    ) from store_error
                emit.message("You are required to re-login before continuing")
            else:
                raise
        except craft_store.errors.CredentialsUnavailable:
            emit.message("You are required to login before continuing")

        self.login()
        return self.store_client.request(*args, **kwargs)

    def register(
        self,
        snap_name: str,
        *,
        is_private: bool = False,
        store_id: Optional[str] = None,
    ) -> None:
        """Register snap_name with the Snap Store.

        :param snap_name: the name of the snap to register with the Snap Store
        :param is_private: makes the registered snap a private snap
        :param store_id: alternative store to register with
        """
        data = dict(
            snap_name=snap_name, is_private=is_private, series=constants.DEFAULT_SERIES
        )
        if store_id is not None:
            data["store"] = store_id

        self.request(
            "POST",
            self._base_url + "/dev/api/register-name/",
            json=data,
        )

    def get_channel_map(self, *, snap_name: str) -> channel_map.ChannelMap:
        """Return the channel map for snap_name."""
        response = self.request(
            "GET",
            self._base_url + f"/api/v2/snaps/{snap_name}/channel-map",
            headers={
                "Accept": "application/json",
            },
        )

        return channel_map.ChannelMap.unmarshal(response.json())

    def get_account_info(
        self,
    ) -> Dict[str, Any]:
        """Return account information."""
        return self.request(
            "GET",
            self._base_url + "/dev/api/account",
            headers={"Accept": "application/json"},
        ).json()

    def get_names(self) -> List[Tuple[str, str, str, str]]:
        """Return a table with the registered names and status."""
        account_info = self.get_account_info()

        snaps: List[Tuple[str, str, str, str]] = [
            (
                name,
                info["since"],
                "private" if info["private"] else "public",
                "-",
            )
            for name, info in account_info["snaps"]
            .get(constants.DEFAULT_SERIES, {})
            .items()
            # Presenting only approved snap registrations, which means name
            # disputes will be displayed/sorted some other way.
            if info["status"] == "Approved"
        ]
        return snaps

    def release(
        self,
        snap_name: str,
        *,
        revision: int,
        channels: Sequence[str],
        progressive_percentage: Optional[int] = None,
    ) -> None:
        """Register snap_name with the Snap Store.

        :param snap_name: the name of the snap to register with the Snap Store
        :param revision: the revision of the snap to release
        :param channels: the channels to release to
        :param progressive_percentage: enable progressive releases up to a given percentage
        """
        data: Dict[str, Any] = {
            "name": snap_name,
            "revision": str(revision),
            "channels": channels,
        }
        if progressive_percentage is not None and progressive_percentage != 100:
            data["progressive"] = {
                "percentage": progressive_percentage,
                "paused": False,
            }
        self.request(
            "POST",
            self._base_url + "/dev/api/snap-release/",
            json=data,
        )

    def close(self, snap_name: str, channel: str) -> None:
        """Close channel for snap_id.

        :param snap_id: the id for the snap to close
        :param channel: the channel to close
        """
        # Account info request to retrieve the snap-id
        account_info = self.get_account_info()
        try:
            snap_id = account_info["snaps"][constants.DEFAULT_SERIES][snap_name][
                "snap-id"
            ]
        except KeyError as key_error:
            emit.debug(f"{key_error!r} no found in {account_info!r}")
            raise errors.SnapcraftError(
                f"{snap_name!r} not found or not owned by this account"
            ) from key_error

        self.request(
            "POST",
            self._base_url + f"/dev/api/snaps/{snap_id}/close",
            json={"channels": [channel]},
        )

    def verify_upload(
        self,
        *,
        snap_name: str,
    ) -> None:
        """Verify if this account can perform an upload for this snap_name."""
        data = {
            "name": snap_name,
            "dry_run": True,
        }
        self.request(
            "POST",
            self._base_url + "/dev/api/snap-push/",
            json=data,
            headers={
                "Accept": "application/json",
            },
        )

    def notify_upload(
        self,
        *,
        snap_name: str,
        upload_id: str,
        snap_file_size: int,
        built_at: Optional[str],
        channels: Optional[Sequence[str]],
    ) -> int:
        """Notify an upload to the Snap Store.

        :param snap_name: name of the snap
        :param upload_id: the upload_id to register with the Snap Store
        :param snap_file_size: the file size of the uploaded snap
        :param built_at: the build timestamp for this build
        :param channels: the channels to release to after being accepted into the Snap Store
        :returns: the snap's processed revision
        """
        data = {
            "name": snap_name,
            "series": constants.DEFAULT_SERIES,
            "updown_id": upload_id,
            "binary_filesize": snap_file_size,
            "source_uploaded": False,
        }
        if built_at is not None:
            data["built_at"] = built_at
        if channels is not None:
            data["channels"] = channels

        response = self.request(
            "POST",
            self._base_url + "/dev/api/snap-push/",
            json=data,
            headers={
                "Accept": "application/json",
            },
        )

        status_url = response.json()["status_details_url"]
        while True:
            response = self.request("GET", status_url)
            status = response.json()
            human_status = _HUMAN_STATUS.get(status["code"], status["code"])
            emit.progress(f"Status: {human_status}")

            if status.get("processed", False):
                if status.get("errors"):
                    error_messages = [
                        e["message"] for e in status["errors"] if "message" in e
                    ]
                    error_string = "\n".join([f"- {e}" for e in error_messages])
                    raise errors.SnapcraftError(
                        f"Issues while processing snap:\n{error_string}"
                    )
                break

            time.sleep(_POLL_DELAY)

        return status["revision"]

    def list_revisions(self, snap_name: str) -> Revisions:
        """Return a list of available revisions for snap_name.

        :param snap_name: the name of the snap to query.
        """
        response = self.request(
            "GET",
            f"{self._base_url}/api/v2/snaps/{snap_name}/releases",
            headers={
                "Content-Type": "application/json",
                "Accept": "application/json",
            },
        )

        return Revisions.unmarshal(response.json())


class OnPremStoreClientCLI(LegacyStoreClientCLI):
    """On Premises Store Client command line interface."""

    @overrides
    def request(self, *args, **kwargs) -> requests.Response:
        return self.store_client.request(*args, **kwargs)

    @overrides
    def verify_upload(
        self,
        *,
        snap_name: str,
    ) -> None:
        emit.debug(f"Skipping verification for {snap_name!r}")

    @overrides
    def notify_upload(
        self,
        *,
        snap_name: str,
        upload_id: str,
        snap_file_size: int,
        built_at: Optional[str],
        channels: Optional[Sequence[str]],
    ) -> int:
        if channels:
            raise errors.SnapcraftError("Releasing during currently unsupported")
        emit.debug(
            f"Ignoring snap_file_size of {snap_file_size!r} and "
            f"built_at {built_at!r}"
        )

        revision_request = cast(
            craft_store.models.RevisionsRequestModel,
            craft_store.models.RevisionsRequestModel.unmarshal(
                {"upload-id": upload_id}
            ),
        )
        revision_response = self.store_client.notify_revision(
            name=snap_name, revision_request=revision_request
        )

        status_url = self._base_url + revision_response.status_url
        while True:
            response = self.request("GET", status_url)
            # human_status = _HUMAN_STATUS.get(status["code"], status["code"])
            emit.progress(f"Status checked: {response}")

            (revision,) = response.json()["revisions"]
            status = revision["status"]

            if status == "approved":
                return revision["revision"]
            if status == "rejected":
                # TODO: grab more that the first error
                error = revision["errors"][0]
                raise errors.SnapcraftError(
                    f"Error uploading snap: {error['code']}", details=error["message"]
                )
            time.sleep(_POLL_DELAY)

    @overrides
    def release(
        self,
        snap_name: str,
        *,
        revision: Optional[int],
        channels: Sequence[str],
        progressive_percentage: Optional[int] = None,
    ) -> None:
        if progressive_percentage is not None:
            raise errors.SnapcraftError("Progressive percentage currently unsupported")

        payload = [{"revision": revision, "channel": channel} for channel in channels]
        self.request(
            "POST",
            self._base_url
            + self.store_client._endpoints.get_releases_endpoint(  # pylint: disable=protected-access
                snap_name
            ),
            json=payload,
        )

    @overrides
    def close(self, snap_name: str, channel) -> None:
        self.release(snap_name=snap_name, revision=None, channels=[channel])

    @overrides
    def get_channel_map(self, *, snap_name: str) -> channel_map.ChannelMap:
        response = self.request(
            "GET",
            self._base_url
            + self.store_client._endpoints.get_releases_endpoint(  # pylint: disable=protected-access
                snap_name
            ),
        )

        return channel_map.ChannelMap.from_list_releases(
            cast(
                craft_store.models.SnapListReleasesModel,
                craft_store.models.SnapListReleasesModel.unmarshal(response.json()),
            )
        )

    @overrides
    def list_revisions(self, snap_name: str) -> Revisions:
        response = self.request(
            "GET",
            f"{self._base_url}/v1/snap/{snap_name}/revisions",
            headers={
                "Content-Type": "application/json",
                "Accept": "application/json",
            },
        )

        return Revisions.unmarshal(response.json())


# We have two stores with a rather different implementation.
# Define the correct client to use on module load in order
# to not change the entire code base all at once.
class StoreClientCLI(LegacyStoreClientCLI):
    """StoreClientCLI factory."""

    def __new__(cls, ephemeral: bool = False):
        """Return a new instance depending on the store implementation."""
        if is_onprem():
            return OnPremStoreClientCLI(ephemeral)
        return super().__new__(cls)
