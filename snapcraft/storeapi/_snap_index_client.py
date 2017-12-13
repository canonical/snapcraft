import contextlib
import os
from typing import Dict

from ._client import Client

from . import errors
from . import logger, _macaroon_auth
from . import constants


class SnapIndexClient(Client):
    """The Click Package Index knows everything about existing snaps.
    https://wiki.ubuntu.com/AppStore/Interfaces/ClickPackageIndex is the
    canonical reference.
    """

    def __init__(self, conf):
        super().__init__(conf, os.environ.get(
            'UBUNTU_STORE_SEARCH_ROOT_URL',
            constants.UBUNTU_STORE_SEARCH_ROOT_URL))

    def get_default_headers(self):
        """Return default headers for CPI requests.
        Tries to build an 'Authorization' header with local credentials
        if they are available.
        Also pin specific branded store if `SNAPCRAFT_UBUNTU_STORE`
        environment is set.
        """
        headers = {}

        with contextlib.suppress(errors.InvalidCredentialsError):
            headers['Authorization'] = _macaroon_auth(self.conf)

        branded_store = os.getenv('SNAPCRAFT_UBUNTU_STORE')
        if branded_store:
            headers['X-Ubuntu-Store'] = branded_store

        return headers

    def get_package(self, snap_name, channel, arch=None):
        headers = self.get_default_headers()
        headers.update({
            'Accept': 'application/hal+json',
            'X-Ubuntu-Series': constants.DEFAULT_SERIES,
        })
        if arch:
            headers['X-Ubuntu-Architecture'] = arch

        params = {
            'channel': channel,
            # FIXME LP: #1662665
            'fields': 'status,anon_download_url,download_url,'
                      'download_sha3_384,download_sha512,snap_id,'
                      'revision,release',
        }
        logger.debug('Getting details for {}'.format(snap_name))
        url = 'api/v1/snaps/details/{}'.format(snap_name)
        resp = self.get(url, headers=headers, params=params)
        if resp.status_code != 200:
            raise errors.SnapNotFoundError(snap_name, channel, arch)
        return resp.json()

    def get_assertion(self, assertion_type: str,
                      snap_id: str) -> Dict[str, Dict[str, str]]:
        headers = self.get_default_headers()
        logger.debug('Getting snap-declaration for {}'.format(snap_id))
        url = '/api/v1/snaps/assertions/{}/{}/{}'.format(
            assertion_type, constants.DEFAULT_SERIES, snap_id)
        response = self.get(url, headers=headers)
        if response.status_code != 200:
            raise errors.SnapNotFoundError(
                snap_id, series=constants.DEFAULT_SERIES)
        return response.json()

    def get(self, url, headers=None, params=None, stream=False):
        if headers is None:
            headers = self.get_default_headers()
        response = self.request('GET', url, stream=stream,
                                headers=headers, params=params)
        return response
