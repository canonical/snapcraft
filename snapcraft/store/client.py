# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022,2024 Canonical Ltd.
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
from collections.abc import Sequence
from datetime import timedelta
from typing import Any

import craft_store
import distro
import pydantic
import requests
from craft_application.util.error_formatting import format_pydantic_errors
from craft_cli import emit
from craft_platforms import DebianArchitecture
from overrides import overrides

from snapcraft import __version__, errors, models, utils
from snapcraft_legacy.storeapi.v2.releases import Releases as Revisions

from . import channel_map, constants
from ._legacy_account import LegacyUbuntuOne
from .onprem_client import ON_PREM_ENDPOINTS, OnPremClient

_POLL_DELAY = 1
_HUMAN_STATUS = {
    "being_processed": "processing",
    "ready_to_release": "ready to release!",
    "need_manual_review": "will need manual review",
    "processing_upload_delta_error": "error while processing delta",
    "processing_error": "error while processing",
}


def build_user_agent(version: str = __version__):
    """Build Snapcraft's user agent."""
    dist_id = distro.id()
    dist_version = distro.version()
    dist_arch = DebianArchitecture.from_host().to_platform_arch()
    return f"snapcraft/{version} {dist_id}/{dist_version} ({dist_arch})"


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


def _prompt_login() -> tuple[str, str]:
    emit.message("Enter your Ubuntu One e-mail address and password.")
    emit.message(
        "If you do not have an Ubuntu One account, you can create one "
        "at https://snapcraft.io/account",
    )
    email = utils.prompt("Email: ")
    password = utils.prompt("Password: ", hide=True)

    return (email, password)


def _get_hostname(
    hostname: str | None = platform.node(),  # noqa: B008 Function call in arg defaults
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

    def __init__(self, ephemeral: bool = False):
        self.store_client = get_client(ephemeral=ephemeral)
        self._base_url = get_store_url()

    def login(
        self,
        *,
        ttl: int = int(timedelta(days=365).total_seconds()),  # noqa: B008
        acls: Sequence[str] | None = None,
        packages: Sequence[str] | None = None,
        channels: Sequence[str] | None = None,
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
            if store_error.response.status_code == requests.codes.unauthorized:
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
        store_id: str | None = None,
    ) -> None:
        """Register snap_name with the Snap Store.

        :param snap_name: the name of the snap to register with the Snap Store
        :param is_private: makes the registered snap a private snap
        :param store_id: alternative store to register with
        """
        data = {
            "snap_name": snap_name,
            "is_private": is_private,
            "series": constants.DEFAULT_SERIES,
        }
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
    ) -> dict[str, Any]:
        """Return account information."""
        return self.request(
            "GET",
            self._base_url + "/dev/api/account",
            headers={"Accept": "application/json"},
        ).json()

    def get_names(self) -> list[tuple[str, str, str, str]]:
        """Return a table with the registered names and status."""
        account_info = self.get_account_info()

        snaps: list[tuple[str, str, str, str]] = [
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
        progressive_percentage: int | None = None,
    ) -> None:
        """Register snap_name with the Snap Store.

        :param snap_name: the name of the snap to register with the Snap Store
        :param revision: the revision of the snap to release
        :param channels: the channels to release to
        :param progressive_percentage: enable progressive releases up to a given percentage
        """
        data: dict[str, Any] = {
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

    def notify_upload(  # noqa: PLR0913 (too-many-arguments)
        self,
        *,
        snap_name: str,
        upload_id: str,
        snap_file_size: int,
        built_at: str | None,
        channels: Sequence[str] | None,
        components: dict[str, str] | None,
    ) -> int:
        """Notify an upload to the Snap Store.

        :param snap_name: name of the snap
        :param upload_id: the upload_id to register with the Snap Store
        :param snap_file_size: the file size of the uploaded snap
        :param built_at: the build timestamp for this build
        :param channels: the channels to release to after being accepted into the Snap Store
        :param components: A dictionary of component names to component upload-ids.
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
        if components:
            data["components"] = components

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

    @staticmethod
    def _unmarshal_confdb_schema(
        confdb_schema_data: dict[str, Any],
    ) -> models.ConfdbSchemaAssertion:
        """Unmarshal a confdb schema.

        :raises StoreAssertionError: If the confdb schema cannot be unmarshalled.
        """
        try:
            return models.ConfdbSchemaAssertion.unmarshal(confdb_schema_data)
        except pydantic.ValidationError as err:
            raise errors.SnapcraftAssertionError(
                message="Received invalid confdb schema from the store",
                # this is an unexpected failure that the user can't fix, so hide
                # the response in the details
                details=f"{format_pydantic_errors(err.errors(), file_name='confdb schema')}",
            ) from err

    def list_confdb_schemas(
        self, *, name: str | None = None
    ) -> list[models.ConfdbSchemaAssertion]:
        """Return a list of confdb schemas.

        :param name: If specified, only list the confdb schema with that name.
        """
        endpoint = f"{self._base_url}/api/v2/confdb-schemas"
        if name:
            endpoint += f"/{name}"

        response = self.request(
            "GET",
            endpoint,
            headers={
                "Content-Type": "application/json",
                "Accept": "application/json",
            },
        )

        confdb_assertions = []
        if assertions := response.json().get("assertions"):
            for assertion_data in assertions:
                # move body into model
                assertion_data["headers"]["body"] = assertion_data.get("body")

                assertion = self._unmarshal_confdb_schema(assertion_data["headers"])
                confdb_assertions.append(assertion)
                emit.debug(f"Parsed confdb schema: {assertion.model_dump_json()}")

        return confdb_assertions

    def build_confdb_schema(
        self, *, confdb_schema: models.EditableConfdbSchemaAssertion
    ) -> models.ConfdbSchemaAssertion:
        """Build a confdb schema.

        Sends an edited confdb schema to the store, which validates the data,
        populates additional fields, and returns the confdb schema.

        :param confdb_schema: The confdb schema to build.

        :returns: The built confdb schema.
        """
        response = self.request(
            "POST",
            f"{self._base_url}/api/v2/confdb-schemas/build-assertion",
            headers={
                "Content-Type": "application/json",
                "Accept": "application/json",
            },
            json=confdb_schema.marshal(),
        )

        assertion = self._unmarshal_confdb_schema(response.json())
        emit.debug(f"Built confdb schema: {assertion.model_dump_json()}")
        return assertion

    def post_confdb_schema(
        self, *, confdb_schema_data: bytes
    ) -> models.ConfdbSchemaAssertion:
        """Send a confdb schema to be published.

        :param confdb_schema_data: A signed confdb schema represented as bytes.

        :returns: The published assertion.
        """
        response = self.request(
            "POST",
            f"{self._base_url}/api/v2/confdb-schemas",
            headers={
                "Accept": "application/json",
                "Content-Type": "application/x.ubuntu.assertion",
            },
            data=confdb_schema_data,
        )

        assertions = response.json().get("assertions")

        if not assertions or len(assertions) != 1:
            raise errors.SnapcraftAssertionError(
                message="Received invalid confdb schema from the store",
                # this is an unexpected failure that the user can't fix, so hide
                # the response in the details
                details=f"Received data: {assertions}",
            )

        # move body into model
        assertions[0]["headers"]["body"] = assertions[0]["body"]

        assertion = self._unmarshal_confdb_schema(assertions[0]["headers"])
        emit.debug(f"Published confdb schema: {assertion.model_dump_json()}")
        return assertion


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
    def notify_upload(  # noqa: PLR0913 (too-many-arguments)
        self,
        *,
        snap_name: str,
        upload_id: str,
        snap_file_size: int,
        built_at: str | None,
        channels: Sequence[str] | None,
        components: dict[str, str] | None,
    ) -> int:
        if channels:
            raise errors.SnapcraftError("Releasing during currently unsupported")
        if components:
            raise errors.SnapcraftError(
                "Components are currently unsupported for on-prem stores"
            )
        emit.debug(
            f"Ignoring snap_file_size of {snap_file_size!r} and built_at {built_at!r}"
        )

        revision_request = craft_store.models.RevisionsRequestModel.unmarshal(
            {"upload-id": upload_id}
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
        revision: int | None,
        channels: Sequence[str],
        progressive_percentage: int | None = None,
    ) -> None:
        if progressive_percentage is not None:
            raise errors.SnapcraftError("Progressive percentage currently unsupported")

        payload = [{"revision": revision, "channel": channel} for channel in channels]
        self.request(
            "POST",
            self._base_url
            + self.store_client._endpoints.get_releases_endpoint(snap_name),
            json=payload,
        )

    @overrides
    def close(self, snap_name: str, channel: str) -> None:
        self.release(snap_name=snap_name, revision=None, channels=[channel])

    @overrides
    def get_channel_map(self, *, snap_name: str) -> channel_map.ChannelMap:
        response = self.request(
            "GET",
            self._base_url
            + self.store_client._endpoints.get_releases_endpoint(snap_name),
        )

        return channel_map.ChannelMap.from_list_releases(
            craft_store.models.SnapListReleasesModel.unmarshal(response.json())
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
