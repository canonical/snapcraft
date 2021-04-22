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

import logging
import os
from time import sleep
from typing import Any, Dict, Iterable, List, Optional, TextIO, Union

import requests

from snapcraft.internal.indicators import download_requests_stream
from . import _upload, errors, http_clients
from ._dashboard_api import DashboardAPI
from ._snap_api import SnapAPI
from ._up_down_client import UpDownClient
from .constants import DEFAULT_SERIES
from .v2 import channel_map, releases, validation_sets, whoami


logger = logging.getLogger(__name__)


class StoreClient:
    """High-level client Snap resources."""

    @property
    def use_candid(self) -> bool:
        return isinstance(self.auth_client, http_clients.CandidClient)

    def __init__(self, use_candid: bool = False) -> None:
        super().__init__()

        self.client = http_clients.Client()

        candid_has_credentials = http_clients.CandidClient.has_credentials()
        logger.debug(
            f"Candid forced: {use_candid}. Candid crendendials: {candid_has_credentials}."
        )
        if use_candid or candid_has_credentials:
            self.auth_client: http_clients.AuthClient = http_clients.CandidClient()
        else:
            self.auth_client = http_clients.UbuntuOneAuthClient()

        self.snap = SnapAPI(self.client)
        self.dashboard = DashboardAPI(self.auth_client)
        self._updown = UpDownClient(self.client)

    def login(
        self,
        *,
        acls: Iterable[str] = None,
        channels: Iterable[str] = None,
        packages: Iterable[Dict[str, str]] = None,
        expires: str = None,
        config_fd: TextIO = None,
        **kwargs,
    ) -> None:
        if config_fd is not None:
            return self.auth_client.login(config_fd=config_fd, **kwargs)

        if acls is None:
            acls = [
                "package_access",
                "package_manage",
                "package_push",
                "package_register",
                "package_release",
                "package_update",
            ]

        macaroon = self.dashboard.get_macaroon(
            acls=acls, packages=packages, channels=channels, expires=expires,
        )
        self.auth_client.login(macaroon=macaroon, **kwargs)

    def export_login(self, *, config_fd: TextIO, encode=False) -> None:
        self.auth_client.export_login(config_fd=config_fd, encode=encode)

    def logout(self):
        self.auth_client.logout()

    def whoami(self) -> whoami.WhoAmI:
        """Return user relevant login information."""
        return self.dashboard.whoami()

    def acl(self) -> Dict[str, Any]:
        """Return permissions for the logged-in user."""

        acl_data = {}

        acl_info = self.dashboard.verify_acl()
        for key in ("snap_ids", "channels", "permissions", "expires"):
            acl_data[key] = acl_info.get(key)

        return acl_data

    def get_snap_name_for_id(self, snap_id: str) -> str:
        declaration_assertion = self.snap.get_assertion("snap-declaration", snap_id)
        return declaration_assertion["headers"]["snap-name"]

    def verify_acl(self) -> Dict[str, Union[List[str], str]]:
        return self.dashboard.verify_acl()

    def get_account_information(self):
        return self.dashboard.get_account_information()

    def register_key(self, account_key_request):
        return self.dashboard.register_key(account_key_request)

    def register(self, snap_name: str, is_private: bool = False, store_id: str = None):
        return self.dashboard.register(
            snap_name, is_private=is_private, store_id=store_id, series=DEFAULT_SERIES,
        )

    def upload_precheck(self, snap_name):
        return self.dashboard.snap_upload_precheck(snap_name)

    def push_snap_build(self, snap_id, snap_build):
        return self.dashboard.push_snap_build(snap_id, snap_build)

    def upload(
        self,
        snap_name,
        snap_filename,
        delta_format=None,
        source_hash=None,
        target_hash=None,
        delta_hash=None,
        built_at=None,
        channels: Optional[List[str]] = None,
    ):
        updown_data = _upload.upload_files(snap_filename, self._updown)

        return self.dashboard.snap_upload_metadata(
            snap_name,
            updown_data,
            delta_format=delta_format,
            source_hash=source_hash,
            target_hash=target_hash,
            delta_hash=delta_hash,
            built_at=built_at,
            channels=channels,
        )

    def release(
        self,
        snap_name,
        revision,
        channels,
        progressive_percentage: Optional[int] = None,
    ):
        return self.dashboard.snap_release(
            snap_name,
            revision,
            channels,
            progressive_percentage=progressive_percentage,
        )

    def get_snap_status(self, snap_name, arch=None):
        account_info = self.get_account_information()
        try:
            snap_id = account_info["snaps"][DEFAULT_SERIES][snap_name]["snap-id"]
        except KeyError:
            raise errors.SnapNotFoundError(snap_name=snap_name, arch=arch)

        if snap_id is None:
            raise errors.NoSnapIdError(snap_name)

        response = self.dashboard.snap_status(snap_id, DEFAULT_SERIES, arch)

        if not response:
            raise errors.SnapNotFoundError(snap_name=snap_name, arch=arch)

        return response

    def get_snap_channel_map(self, *, snap_name: str) -> channel_map.ChannelMap:
        return self.dashboard.get_snap_channel_map(snap_name=snap_name)

    def get_snap_releases(self, *, snap_name: str) -> releases.Releases:
        return self.dashboard.get_snap_releases(snap_name=snap_name)

    def post_validation_sets_build_assertion(
        self, *, validation_sets: Dict[str, Any]
    ) -> validation_sets.BuildAssertion:
        return self.dashboard.post_validation_sets_build_assertion(validation_sets)

    def post_validation_sets(
        self, *, signed_validation_sets: bytes
    ) -> validation_sets.ValidationSets:
        return self.dashboard.post_validation_sets(signed_validation_sets)

    def get_validation_sets(
        self, *, name: Optional[str] = None, sequence: Optional[str] = None
    ) -> validation_sets.ValidationSets:
        return self.dashboard.get_validation_sets(name=name, sequence=sequence)

    def close_channels(self, snap_id, channel_names):
        return self.dashboard.close_channels(snap_id, channel_names)

    def download(
        self,
        snap_name,
        *,
        risk: str,
        download_path: str,
        track: Optional[str] = None,
        arch: Optional[str] = None,
        except_hash: str = "",
    ):
        snap_info = self.snap.get_info(snap_name)
        channel_mapping = snap_info.get_channel_mapping(
            risk=risk, track=track, arch=arch
        )
        if channel_mapping.download.sha3_384 == except_hash:
            return channel_mapping.download.sha3_384

        try:
            channel_mapping.download.verify(download_path)
        except errors.StoreDownloadError:
            self._download_snap(channel_mapping.download, download_path)

        channel_mapping.download.verify(download_path)
        return channel_mapping.download.sha3_384

    def _download_snap(self, download_details, download_path):
        # we only resume when redirected to our CDN since we use internap's
        # special sauce.
        total_read = 0
        probe_url = requests.head(download_details.url)
        if probe_url.is_redirect and "internap" in probe_url.headers["Location"]:
            download_url = probe_url.headers["Location"]
            resume_possible = True
        else:
            download_url = download_details.url
            resume_possible = False

        # HttpAdapter cannot help here as this is a stream.
        # LP: #1617765
        not_downloaded = True
        retry_count = 5
        while not_downloaded and retry_count:
            headers = {}
            if resume_possible and os.path.exists(download_path):
                total_read = os.path.getsize(download_path)
                headers["Range"] = "bytes={}-".format(total_read)
            request = self.client.request(
                "GET", download_url, headers=headers, stream=True
            )
            request.raise_for_status()
            redirections = [h.headers["Location"] for h in request.history]
            if redirections:
                logger.debug(
                    "Redirections for {!r}: {}".format(
                        download_url, ", ".join(redirections)
                    )
                )
            try:
                download_requests_stream(request, download_path, total_read=total_read)
                not_downloaded = False
            except requests.exceptions.ChunkedEncodingError as e:
                logger.debug(
                    "Error while downloading: {!r}. "
                    "Retries left to download: {!r}.".format(e, retry_count)
                )
                retry_count -= 1
                if not retry_count:
                    raise e
                sleep(1)

    def push_assertion(self, snap_id, assertion, endpoint, force=False):
        return self.dashboard.push_assertion(snap_id, assertion, endpoint, force)

    def get_assertion(self, snap_id, endpoint, params=None):
        return self.dashboard.get_assertion(snap_id, endpoint, params=params)

    def sign_developer_agreement(self, latest_tos_accepted=False):
        return self.dashboard.sign_developer_agreement(latest_tos_accepted)

    def upload_metadata(self, snap_name, metadata, force):
        """Upload the metadata to the server."""
        account_info = self.get_account_information()
        try:
            snap_id = account_info["snaps"][DEFAULT_SERIES][snap_name]["snap-id"]
        except KeyError:
            raise errors.SnapNotFoundError(snap_name=snap_name)

        if snap_id is None:
            raise errors.NoSnapIdError(snap_name)

        return self.dashboard.upload_metadata(snap_id, snap_name, metadata, force)

    def upload_binary_metadata(self, snap_name, metadata, force):
        """Upload the binary metadata to the server."""
        account_info = self.get_account_information()
        try:
            snap_id = account_info["snaps"][DEFAULT_SERIES][snap_name]["snap-id"]
        except KeyError:
            raise errors.SnapNotFoundError(snap_name=snap_name)

        if snap_id is None:
            raise errors.NoSnapIdError(snap_name)

        return self.dashboard.upload_binary_metadata(
            snap_id, snap_name, metadata, force
        )
