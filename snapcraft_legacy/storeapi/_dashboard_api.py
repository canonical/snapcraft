# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2021 Canonical Ltd
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

import json
import logging
import os
from typing import Any, Dict, List, Optional
from urllib.parse import urlencode, urljoin
import craft_store

import requests
from craft_store import BaseClient, UbuntuOneStoreClient
from simplejson.scanner import JSONDecodeError

from . import _metadata, constants, errors, metrics
from ._requests import Requests
from ._status_tracker import StatusTracker
from .v2 import channel_map, releases, validation_sets, whoami

logger = logging.getLogger(__name__)


class DashboardAPI(Requests):
    """The Dashboard API is used to publish and manage snaps.

    This is an interface to query that API which is documented
    at https://dashboard.snapcraft.io/docs/.
    """

    def __init__(self, auth_client: BaseClient) -> None:
        super().__init__()

        self._auth_client = auth_client

    def _request(self, method: str, urlpath: str, **kwargs) -> requests.Response:
        url = urljoin(self._auth_client._base_url, urlpath)
        response = self._auth_client.request(method, url, **kwargs)
        logger.debug("Call to %s returned: %s", url, response.text)
        return response

    def verify_acl(self):
        if not isinstance(self._auth_client, UbuntuOneStoreClient):
            raise NotImplementedError("Only supports UbuntuOneAuthClient.")

        try:
            response = self.post(
                "/dev/api/acl/verify/",
                json={
                    "auth_data": {
                        "authorization": self._auth_client._auth.get_credentials()
                    }
                },
                headers={"Accept": "application/json"},
            )
        except craft_store.errors.StoreServerError as store_error:
            raise errors.StoreAccountInformationError(
                store_error.response
            ) from store_error

        return response.json()

    def get_account_information(self) -> Dict[str, Any]:
        try:
            response = self.get(
                "/dev/api/account", headers={"Accept": "application/json"}
            )
        except craft_store.errors.StoreServerError as store_error:
            raise errors.StoreAccountInformationError(
                store_error.response
            ) from store_error

        return response.json()

    def register_key(self, account_key_request):
        data = {"account_key_request": account_key_request}
        try:
            response = self.post(
                "/dev/api/account/account-key",
                data=json.dumps(data),
                headers={
                    "Content-Type": "application/json",
                    "Accept": "application/json",
                },
            )
        except craft_store.errors.StoreServerError as store_error:
            raise errors.StoreKeyRegistrationError(
                store_error.response
            ) from store_error

    def register(
        self, snap_name: str, *, is_private: bool, series: str, store_id: Optional[str]
    ) -> None:
        data = dict(snap_name=snap_name, is_private=is_private, series=series)
        if store_id is not None:
            data["store"] = store_id
        try:
            response = self.post(
                "/dev/api/register-name/",
                data=json.dumps(data),
                headers={"Content-Type": "application/json"},
            )
        except craft_store.errors.StoreServerError as store_error:
            raise errors.StoreRegistrationError(
                snap_name, store_error.response
            ) from store_error

    def snap_upload_precheck(self, snap_name):
        data = {"name": snap_name, "dry_run": True}
        try:
            response = self.post(
                "/dev/api/snap-push/",
                data=json.dumps(data),
                headers={
                    "Content-Type": "application/json",
                    "Accept": "application/json",
                },
            )
        except craft_store.errors.StoreServerError as store_error:
            raise errors.StoreUploadError(
                snap_name, store_error.response
            ) from store_error

    def snap_upload_metadata(
        self,
        snap_name,
        updown_data,
        delta_format=None,
        delta_hash=None,
        source_hash=None,
        target_hash=None,
        built_at=None,
        channels: Optional[List[str]] = None,
    ) -> StatusTracker:
        data = {
            "name": snap_name,
            "series": constants.DEFAULT_SERIES,
            "updown_id": updown_data["upload_id"],
            "binary_filesize": updown_data["binary_filesize"],
            "source_uploaded": updown_data["source_uploaded"],
        }

        if delta_format:
            data["delta_format"] = delta_format
            data["delta_hash"] = delta_hash
            data["source_hash"] = source_hash
            data["target_hash"] = target_hash
        if built_at is not None:
            data["built_at"] = built_at
        if channels is not None:
            data["channels"] = channels
        try:
            response = self.post(
                "/dev/api/snap-push/",
                data=json.dumps(data),
                headers={
                    "Content-Type": "application/json",
                    "Accept": "application/json",
                },
            )
        except craft_store.errors.StoreServerError as store_error:
            raise errors.StoreUploadError(
                data["name"], store_error.response
            ) from store_error

        return StatusTracker(response.json()["status_details_url"])

    def upload_metadata(self, snap_id, snap_name, metadata, force):
        """Upload the metadata to SCA."""
        metadata_handler = _metadata.StoreMetadataHandler(
            request_method=self._request,
            snap_id=snap_id,
            snap_name=snap_name,
        )
        metadata_handler.upload(metadata, force)

    def upload_binary_metadata(self, snap_id, snap_name, metadata, force):
        """Upload the binary metadata to SCA."""
        metadata_handler = _metadata.StoreMetadataHandler(
            request_method=self._request,
            snap_id=snap_id,
            snap_name=snap_name,
        )
        metadata_handler.upload_binary(metadata, force)

    def snap_release(
        self,
        snap_name,
        revision,
        channels,
        delta_format=None,
        progressive_percentage: Optional[int] = None,
    ):
        data = {"name": snap_name, "revision": str(revision), "channels": channels}
        if delta_format:
            data["delta_format"] = delta_format
        if progressive_percentage is not None:
            data["progressive"] = {
                "percentage": progressive_percentage,
                "paused": False,
            }
        try:
            response = self.post(
                "/dev/api/snap-release/",
                data=json.dumps(data),
                headers={
                    "Content-Type": "application/json",
                    "Accept": "application/json",
                },
            )
        except craft_store.errors.StoreServerError as store_error:
            raise errors.StoreReleaseError(
                data["name"], store_error.response
            ) from store_error

        response_json = response.json()

        return response_json

    def push_assertion(self, snap_id, assertion, endpoint, force):
        if endpoint == "validations":
            data = {"assertion": assertion.decode("utf-8")}
        elif endpoint == "developers":
            data = {"snap_developer": assertion.decode("utf-8")}
        else:
            raise RuntimeError("No valid endpoint")

        url = "/dev/api/snaps/{}/{}".format(snap_id, endpoint)

        # For `snap-developer`, revoking developers will require their uploads
        # to be invalidated.
        if force:
            url = url + "?ignore_revoked_uploads"

        try:
            response = self.put(
                url,
                json=data,
                headers={
                    "Content-Type": "application/json",
                    "Accept": "application/json",
                },
            )
        except craft_store.errors.StoreServerError as craft_error:
            raise errors.StoreValidationError(
                snap_id, craft_error.response
            ) from craft_error

        try:
            response_json = response.json()
        except JSONDecodeError:
            message = (
                "Invalid response from the server when pushing validations: {} {}"
            ).format(response.status_code, response)
            logger.debug(message)
            raise errors.StoreValidationError(
                snap_id, response, message="Invalid response from the server"
            )

        return response_json

    def get_assertion(self, snap_id, endpoint, params=None):
        try:
            response = self.get(
                f"/dev/api/snaps/{snap_id}/{endpoint}",
                headers={
                    "Content-Type": "application/json",
                    "Accept": "application/json",
                },
                params=params,
            )
        except craft_store.errors.StoreServerError as craft_error:
            raise errors.StoreValidationError(
                snap_id, craft_error.response
            ) from craft_error
        try:
            response_json = response.json()
        except JSONDecodeError:
            message = "Invalid response from the server when getting {}: {} {}".format(
                endpoint, response.status_code, response
            )
            logger.debug(message)
            raise errors.StoreValidationError(
                snap_id, response, message="Invalid response from the server"
            )

        return response_json

    def push_snap_build(self, snap_id, snap_build):
        url = f"/dev/api/snaps/{snap_id}/builds"
        data = json.dumps({"assertion": snap_build})
        headers = {
            "Content-Type": "application/json",
        }
        try:
            self.post(url, data=data, headers=headers)
        except craft_store.errors.StoreServerError as craft_error:
            raise errors.StoreSnapBuildError(craft_error.response) from craft_error

    def snap_status(self, snap_id, series, arch):
        qs = {}
        if series:
            qs["series"] = series
        if arch:
            qs["architecture"] = arch
        url = "/dev/api/snaps/" + snap_id + "/state"
        if qs:
            url += "?" + urlencode(qs)
        try:
            response = self.get(
                url,
                headers={
                    "Content-Type": "application/json",
                    "Accept": "application/json",
                },
            )
        except craft_store.errors.StoreServerError as craft_error:
            raise errors.StoreSnapStatusError(
                craft_error.response, snap_id, series, arch
            ) from craft_error

        response_json = response.json()

        return response_json

    def close_channels(self, snap_id, channel_names):
        url = "/dev/api/snaps/{}/close".format(snap_id)
        data = {"channels": channel_names}
        headers = {"Content-Type": "application/json", "Accept": "application/json"}

        try:
            response = self.post(url, data=json.dumps(data), headers=headers)
        except craft_store.errors.StoreServerError as craft_error:
            raise errors.StoreChannelClosingError(craft_error.response) from craft_error

        try:
            results = response.json()
            return results["closed_channels"], results["channel_map_tree"]
        except (JSONDecodeError, KeyError):
            logger.debug(
                "Invalid response from the server on channel closing:\n"
                "{} {}\n{}".format(
                    response.status_code, response.reason, response.content
                )
            )
            raise errors.StoreChannelClosingError(response)

    def sign_developer_agreement(self, latest_tos_accepted=False):
        data = {"latest_tos_accepted": latest_tos_accepted}

        try:
            response = self.post(
                "/dev/api/agreement/",
                json=data,
                headers={
                    "Content-Type": "application/json",
                    "Accept": "application/json",
                },
            )
        except craft_store.errors.StoreServerError as store_error:
            raise errors.DeveloperAgreementSignError(
                store_error.response
            ) from store_error

        return response.json()

    def get_snap_channel_map(self, *, snap_name: str) -> channel_map.ChannelMap:
        try:
            response = self.get(
                f"/api/v2/snaps/{snap_name}/channel-map",
                headers={
                    "Content-Type": "application/json",
                    "Accept": "application/json",
                },
            )
        except craft_store.errors.StoreServerError as store_error:
            raise errors.StoreSnapChannelMapError(snap_name=snap_name) from store_error

        return channel_map.ChannelMap.unmarshal(response.json())

    def get_metrics(
        self, filters: List[metrics.MetricsFilter], snap_name: str
    ) -> metrics.MetricsResults:
        url = "/dev/api/snaps/metrics"
        data = {"filters": [f.marshal() for f in filters]}
        headers = {"Content-Type": "application/json", "Accept": "application/json"}

        try:
            response = self.post(url, data=json.dumps(data), headers=headers)

        except craft_store.errors.StoreServerError as store_error:
            raise errors.StoreMetricsError(
                filters=filters, response=store_error.response, snap_name=snap_name
            ) from store_error

        try:
            results = response.json()
            return metrics.MetricsResults.unmarshal(results)
        except ValueError as error:
            raise errors.StoreMetricsUnmarshalError(
                filters=filters, snap_name=snap_name, response=response
            ) from error

    def get_snap_releases(self, *, snap_name: str) -> releases.Releases:
        try:
            response = self.get(
                f"/api/v2/snaps/{snap_name}/releases",
                headers={
                    "Content-Type": "application/json",
                    "Accept": "application/json",
                },
            )
        except craft_store.errors.StoreServerError as store_error:
            raise errors.StoreSnapChannelMapError(snap_name=snap_name) from store_error

        return releases.Releases.unmarshal(response.json())

    def whoami(self) -> whoami.WhoAmI:
        try:
            response = self.get(
                "/api/v2/tokens/whoami",
                headers={
                    "Content-Type": "application/json",
                    "Accept": "application/json",
                },
            )
        except craft_store.errors.StoreServerError as store_error:
            raise errors.GeneralStoreError(
                message="whoami failed.", response=store_error.response
            ) from store_error

        return whoami.WhoAmI.unmarshal(response.json())

    def post_validation_sets_build_assertion(
        self, validation_sets_data: Dict[str, Any]
    ) -> validation_sets.BuildAssertion:
        url = "/api/v2/validation-sets/build-assertion"
        try:
            response = self.post(
                url,
                headers={
                    "Accept": "application/json",
                    "Content-Type": "application/json",
                },
                json=validation_sets_data,
            )
        except craft_store.errors.StoreServerError as store_error:
            raise errors.StoreValidationSetsError(store_error.response) from store_error

        return validation_sets.BuildAssertion.unmarshal(response.json())

    def post_validation_sets(
        self, signed_validation_sets: bytes
    ) -> validation_sets.ValidationSets:
        url = "/api/v2/validation-sets"
        try:
            response = self.post(
                url,
                headers={
                    "Accept": "application/json",
                    "Content-Type": "application/x.ubuntu.assertion",
                },
                data=signed_validation_sets,
            )
        except craft_store.errors.StoreServerError as store_error:
            raise errors.StoreValidationSetsError(store_error.response) from store_error

        return validation_sets.ValidationSets.unmarshal(response.json())

    def get_validation_sets(
        self, *, name: Optional[str], sequence: Optional[str]
    ) -> validation_sets.ValidationSets:
        url = "/api/v2/validation-sets"
        if name is not None:
            url += "/" + name
        params = dict()
        if sequence is not None:
            params["sequence"] = sequence

        try:
            response = self.get(
                url, headers={"Accept": "application/json"}, params=params
            )
        except craft_store.errors.StoreServerError as store_error:
            raise errors.StoreValidationSetsError(store_error.response) from store_error

        return validation_sets.ValidationSets.unmarshal(response.json())
