import hashlib
import os
import urllib.parse
from time import sleep
from typing import Dict, Iterable, List, TextIO, Union

import pymacaroons
import requests

import snapcraft
from snapcraft import config
from snapcraft.internal.indicators import download_requests_stream

from . import logger
from . import _upload
from . import constants
from . import errors

from ._sso_client import SSOClient
from ._snap_index_client import SnapIndexClient
from ._sca_client import SCAClient
from ._up_down_client import UpDownClient


class StoreClient:
    """High-level client for the V2.0 API SCA resources."""

    def __init__(self) -> None:
        super().__init__()
        self.conf = config.Config()
        self.sso = SSOClient(self.conf)
        self.cpi = SnapIndexClient(self.conf)
        self.updown = UpDownClient(self.conf)
        self.sca = SCAClient(self.conf)

    def login(
        self,
        email: str,
        password: str,
        one_time_password: str = None,
        acls: Iterable[str] = None,
        channels: Iterable[str] = None,
        packages: Iterable[Dict[str, str]] = None,
        expires: str = None,
        config_fd: TextIO = None,
        save: bool = True,
    ) -> None:
        """Log in via the Ubuntu One SSO API."""
        if acls is None:
            acls = [
                "package_access",
                "package_manage",
                "package_push",
                "package_register",
                "package_release",
                "package_update",
            ]

        if config_fd:
            self.conf.load(config_fd=config_fd)
        else:
            # Ask the store for the needed capabilities to be associated with
            # the macaroon.
            macaroon = self.sca.get_macaroon(acls, packages, channels, expires)
            caveat_id = self._extract_caveat_id(macaroon)
            unbound_discharge = self.sso.get_unbound_discharge(
                email, password, one_time_password, caveat_id
            )
            # The macaroon has been discharged, save it in the config
            self.conf.set("macaroon", macaroon)
            self.conf.set("unbound_discharge", unbound_discharge)
            self.conf.set("email", email)

        if save:
            self.conf.save()

    def _extract_caveat_id(self, root_macaroon):
        macaroon = pymacaroons.Macaroon.deserialize(root_macaroon)
        # macaroons are all bytes, never strings
        sso_host = urllib.parse.urlparse(self.sso.root_url).netloc
        for caveat in macaroon.caveats:
            if caveat.location == sso_host:
                return caveat.caveat_id
        else:
            raise errors.InvalidCredentialsError("Invalid root macaroon")

    def logout(self):
        self.conf.clear()
        self.conf.save()

    def _refresh_if_necessary(self, func, *args, **kwargs):
        """Make a request, refreshing macaroons if necessary."""
        try:
            return func(*args, **kwargs)
        except errors.StoreMacaroonNeedsRefreshError:
            unbound_discharge = self.sso.refresh_unbound_discharge(
                self.conf.get("unbound_discharge")
            )
            self.conf.set("unbound_discharge", unbound_discharge)
            self.conf.save()
            return func(*args, **kwargs)

    def whoami(self):
        """Return user relevant login information."""
        account_data = {}

        for k in ("email", "account_id"):
            value = self.conf.get(k)
            if not value:
                account_info = self.get_account_information()
                value = account_info.get(k, "unknown")
                self.conf.set(k, value)
                self.conf.save()
            account_data[k] = value

        return account_data

    def acl(self) -> Dict[str, Union[List[str], str]]:
        """Return permissions for the logged-in user."""

        acl_data = {}

        acl_info = self.verify_acl()
        for key in ("snap_ids", "channels", "permissions", "expires"):
            acl_data[key] = acl_info.get(key)

        return acl_data

    def get_snap_name_for_id(self, snap_id: str) -> str:
        declaration_assertion = self.cpi.get_assertion("snap-declaration", snap_id)
        return declaration_assertion["headers"]["snap-name"]

    def verify_acl(self) -> Dict[str, Union[List[str], str]]:
        return self._refresh_if_necessary(self.sca.verify_acl)

    def get_account_information(self):
        return self._refresh_if_necessary(self.sca.get_account_information)

    def register_key(self, account_key_request):
        return self._refresh_if_necessary(self.sca.register_key, account_key_request)

    def register(self, snap_name, is_private=False):
        return self._refresh_if_necessary(
            self.sca.register, snap_name, is_private, constants.DEFAULT_SERIES
        )

    def push_precheck(self, snap_name):
        return self._refresh_if_necessary(self.sca.snap_push_precheck, snap_name)

    def push_snap_build(self, snap_id, snap_build):
        return self._refresh_if_necessary(self.sca.push_snap_build, snap_id, snap_build)

    def upload(
        self,
        snap_name,
        snap_filename,
        delta_format=None,
        source_hash=None,
        target_hash=None,
        delta_hash=None,
    ):
        # FIXME This should be raised by the function that uses the
        # discharge. --elopio -2016-06-20
        if self.conf.get("unbound_discharge") is None:
            raise errors.InvalidCredentialsError(
                "Unbound discharge not in the config file"
            )

        updown_data = _upload.upload_files(snap_filename, self.updown)

        return self._refresh_if_necessary(
            self.sca.snap_push_metadata,
            snap_name,
            updown_data,
            delta_format=delta_format,
            source_hash=source_hash,
            target_hash=target_hash,
            delta_hash=delta_hash,
        )

    def release(self, snap_name, revision, channels):
        return self._refresh_if_necessary(
            self.sca.snap_release, snap_name, revision, channels
        )

    def get_snap_revisions(self, snap_name, series=None, arch=None):
        if series is None:
            series = constants.DEFAULT_SERIES

        account_info = self.get_account_information()
        try:
            snap_id = account_info["snaps"][series][snap_name]["snap-id"]
        except KeyError:
            raise errors.SnapNotFoundError(snap_name, series=series, arch=arch)

        if snap_id is None:
            raise errors.NoSnapIdError(snap_name)

        response = self._refresh_if_necessary(
            self.sca.snap_revisions, snap_id, series, arch
        )

        if not response:
            raise errors.SnapNotFoundError(snap_name, series=series, arch=arch)

        return response

    def get_snap_status(self, snap_name, series=None, arch=None):
        if series is None:
            series = constants.DEFAULT_SERIES

        account_info = self.get_account_information()
        try:
            snap_id = account_info["snaps"][series][snap_name]["snap-id"]
        except KeyError:
            raise errors.SnapNotFoundError(snap_name, series=series, arch=arch)

        if snap_id is None:
            raise errors.NoSnapIdError(snap_name)

        response = self._refresh_if_necessary(
            self.sca.snap_status, snap_id, series, arch
        )

        if not response:
            raise errors.SnapNotFoundError(snap_name, series=series, arch=arch)

        return response

    def close_channels(self, snap_id, channel_names):
        return self._refresh_if_necessary(
            self.sca.close_channels, snap_id, channel_names
        )

    def download(self, snap_name, channel, download_path, arch=None, except_hash=""):
        if arch is None:
            arch = snapcraft.ProjectOptions().deb_arch

        package = self.cpi.get_package(snap_name, channel, arch)
        if package["download_sha3_384"] != except_hash:
            self._download_snap(
                snap_name,
                channel,
                arch,
                download_path,
                # FIXME LP: #1662665
                package["anon_download_url"],
                package["download_sha512"],
            )
        return package["download_sha3_384"]

    def _download_snap(
        self, name, channel, arch, download_path, download_url, expected_sha512
    ):
        if self._is_downloaded(download_path, expected_sha512):
            logger.info("Already downloaded {} at {}".format(name, download_path))
            return
        logger.info("Downloading {}".format(name))

        # we only resume when redirected to our CDN since we use internap's
        # special sauce.
        resume_possible = False
        total_read = 0
        probe_url = requests.head(download_url)
        if probe_url.is_redirect and "internap" in probe_url.headers["Location"]:
            download_url = probe_url.headers["Location"]
            resume_possible = True

        # HttpAdapter cannot help here as this is a stream.
        # LP: #1617765
        not_downloaded = True
        retry_count = 5
        while not_downloaded and retry_count:
            headers = {}
            if resume_possible and os.path.exists(download_path):
                total_read = os.path.getsize(download_path)
                headers["Range"] = "bytes={}-".format(total_read)
            request = self.cpi.get(download_url, headers=headers, stream=True)
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

        if self._is_downloaded(download_path, expected_sha512):
            logger.info("Successfully downloaded {} at {}".format(name, download_path))
        else:
            raise errors.SHAMismatchError(download_path, expected_sha512)

    def _is_downloaded(self, path, expected_sha512):
        if not os.path.exists(path):
            return False

        file_sum = hashlib.sha512()
        with open(path, "rb") as f:
            for file_chunk in iter(lambda: f.read(file_sum.block_size * 128), b""):
                file_sum.update(file_chunk)
        return expected_sha512 == file_sum.hexdigest()

    def push_assertion(self, snap_id, assertion, endpoint, force=False):
        return self.sca.push_assertion(snap_id, assertion, endpoint, force)

    def get_assertion(self, snap_id, endpoint):
        return self.sca.get_assertion(snap_id, endpoint)

    def sign_developer_agreement(self, latest_tos_accepted=False):
        return self.sca.sign_developer_agreement(latest_tos_accepted)

    def push_metadata(self, snap_name, metadata, force):
        """Push the metadata to the server."""
        account_info = self.get_account_information()
        series = constants.DEFAULT_SERIES
        try:
            snap_id = account_info["snaps"][series][snap_name]["snap-id"]
        except KeyError:
            raise errors.SnapNotFoundError(snap_name, series=series)

        if snap_id is None:
            raise errors.NoSnapIdError(snap_name)

        return self._refresh_if_necessary(
            self.sca.push_metadata, snap_id, snap_name, metadata, force
        )

    def push_binary_metadata(self, snap_name, metadata, force):
        """Push the binary metadata to the server."""
        account_info = self.get_account_information()
        series = constants.DEFAULT_SERIES
        try:
            snap_id = account_info["snaps"][series][snap_name]["snap-id"]
        except KeyError:
            raise errors.SnapNotFoundError(snap_name, series=series)

        if snap_id is None:
            raise errors.NoSnapIdError(snap_name)

        return self._refresh_if_necessary(
            self.sca.push_binary_metadata, snap_id, snap_name, metadata, force
        )
