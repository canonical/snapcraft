# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2016 Canonical Ltd
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

import contextlib
import hashlib
import itertools
import json
import logging
import os
import urllib.parse
from time import sleep
from threading import Thread
from queue import Queue

from progressbar import (
    AnimatedMarker,
    ProgressBar,
    UnknownLength,
)
import pymacaroons
import requests
from simplejson.scanner import JSONDecodeError

import snapcraft
from snapcraft import config
from snapcraft.internal.indicators import download_requests_stream
from snapcraft.storeapi import (
    _upload,
    constants,
    errors,
)


logger = logging.getLogger(__name__)


def _macaroon_auth(conf):
    """Format a macaroon and its associated discharge.

    :return: A string suitable to use in an Authorization header.

    """
    root_macaroon_raw = conf.get('macaroon')
    if root_macaroon_raw is None:
        raise errors.InvalidCredentialsError(
            'Root macaroon not in the config file')
    unbound_raw = conf.get('unbound_discharge')
    if unbound_raw is None:
        raise errors.InvalidCredentialsError(
            'Unbound discharge not in the config file')

    root_macaroon = _deserialize_macaroon(root_macaroon_raw)
    unbound = _deserialize_macaroon(unbound_raw)
    bound = root_macaroon.prepare_for_request(unbound)
    discharge_macaroon_raw = bound.serialize()
    auth = 'Macaroon root={}, discharge={}'.format(
        root_macaroon_raw, discharge_macaroon_raw)
    return auth


def _deserialize_macaroon(value):
    try:
        return pymacaroons.Macaroon.deserialize(value)
    except:
        raise errors.InvalidCredentialsError('Failed to deserialize macaroon')


class Client():
    """A base class to define clients for the ols servers.

    This is a simple wrapper around requests.Session so we inherit all good
    bits while providing a simple point for tests to override when needed.

    """

    def __init__(self, conf, root_url):
        self.conf = conf
        self.root_url = root_url
        self.session = requests.Session()
        self._snapcraft_headers = {
            'X-SNAPCRAFT-VERSION': snapcraft.__version__
        }

    def request(self, method, url, params=None, headers=None, **kwargs):
        """Overriding base class to handle the root url."""
        # Note that url may be absolute in which case 'root_url' is ignored by
        # urljoin.

        if headers:
            headers.update(self._snapcraft_headers)
        else:
            headers = self._snapcraft_headers

        final_url = urllib.parse.urljoin(self.root_url, url)
        response = self.session.request(
            method, final_url, headers=headers,
            params=params, **kwargs)
        return response

    def get(self, url, **kwargs):
        return self.request('GET', url, **kwargs)

    def post(self, url, **kwargs):
        return self.request('POST', url, **kwargs)

    def put(self, url, **kwargs):
        return self.request('PUT', url, **kwargs)


class StoreClient():
    """High-level client for the V2.0 API SCA resources."""

    def __init__(self):
        super().__init__()
        self.conf = config.Config()
        self.sso = SSOClient(self.conf)
        self.cpi = SnapIndexClient(self.conf)
        self.updown = UpDownClient(self.conf)
        self.sca = SCAClient(self.conf)

    def login(self, email, password, one_time_password=None, acls=None,
              packages=None, channels=None, save=True):
        """Log in via the Ubuntu One SSO API."""
        if acls is None:
            acls = ['package_upload', 'package_access']
        # Ask the store for the needed capabilities to be associated with the
        # macaroon.
        macaroon = self.sca.get_macaroon(acls, packages, channels)
        caveat_id = self._extract_caveat_id(macaroon)
        unbound_discharge = self.sso.get_unbound_discharge(
            email, password, one_time_password, caveat_id)
        # The macaroon has been discharged, save it in the config
        self.conf.set('macaroon', macaroon)
        self.conf.set('unbound_discharge', unbound_discharge)
        if save:
            self.conf.save()

    def _extract_caveat_id(self, root_macaroon):
        macaroon = pymacaroons.Macaroon.deserialize(root_macaroon)
        # macaroons are all bytes, never strings
        sso_host = urllib.parse.urlparse(self.sso.root_url).hostname
        for caveat in macaroon.caveats:
            if caveat.location == sso_host:
                return caveat.caveat_id
        else:
            raise errors.InvalidCredentialsError('Invalid root macaroon')

    def logout(self):
        self.conf.clear()
        self.conf.save()

    def _refresh_if_necessary(self, func, *args, **kwargs):
        """Make a request, refreshing macaroons if necessary."""
        try:
            return func(*args, **kwargs)
        except errors.StoreMacaroonNeedsRefreshError:
            unbound_discharge = self.sso.refresh_unbound_discharge(
                self.conf.get('unbound_discharge'))
            self.conf.set('unbound_discharge', unbound_discharge)
            self.conf.save()
            return func(*args, **kwargs)

    def get_account_information(self):
        return self._refresh_if_necessary(self.sca.get_account_information)

    def register_key(self, account_key_request):
        return self._refresh_if_necessary(
            self.sca.register_key, account_key_request)

    def register(self, snap_name, is_private=False):
        return self._refresh_if_necessary(
            self.sca.register, snap_name, is_private, constants.DEFAULT_SERIES)

    def push_snap_build(self, snap_id, snap_build):
        return self._refresh_if_necessary(
            self.sca.push_snap_build, snap_id, snap_build)

    def upload(self, snap_name, snap_filename):
        # FIXME This should be raised by the function that uses the
        # discharge. --elopio -2016-06-20
        if self.conf.get('unbound_discharge') is None:
            raise errors.InvalidCredentialsError(
                'Unbound discharge not in the config file')

        updown_data = _upload.upload_files(snap_filename, self.updown)

        return self._refresh_if_necessary(
            self.sca.snap_push_metadata, snap_name, updown_data)

    def release(self, snap_name, revision, channels):
        return self._refresh_if_necessary(
            self.sca.snap_release, snap_name, revision, channels)

    def get_snap_history(self, snap_name, series=None, arch=None):
        if series is None:
            series = constants.DEFAULT_SERIES

        account_info = self.get_account_information()
        try:
            snap_id = account_info['snaps'][series][snap_name]['snap-id']
        except KeyError:
            raise errors.SnapNotFoundError(snap_name, series=series, arch=arch)

        response = self._refresh_if_necessary(
            self.sca.snap_history, snap_id, series, arch)

        if not response:
            raise errors.SnapNotFoundError(snap_name, series=series, arch=arch)

        return response

    def get_snap_status(self, snap_name, series=None, arch=None):
        if series is None:
            series = constants.DEFAULT_SERIES

        account_info = self.get_account_information()
        try:
            snap_id = account_info['snaps'][series][snap_name]['snap-id']
        except KeyError:
            raise errors.SnapNotFoundError(snap_name, series=series, arch=arch)

        response = self._refresh_if_necessary(
            self.sca.snap_status, snap_id, series, arch)

        if not response:
            raise errors.SnapNotFoundError(snap_name, series=series, arch=arch)

        return response

    def close_channels(self, snap_id, channel_names):
        return self._refresh_if_necessary(
            self.sca.close_channels, snap_id, channel_names)

    def download(self, snap_name, channel, download_path, arch=None):
        if arch is None:
            arch = snapcraft.ProjectOptions().deb_arch

        package = self.cpi.get_package(snap_name, channel, arch)
        self._download_snap(
            snap_name, channel, arch, download_path,
            package['anon_download_url'], package['download_sha512'])

    def _download_snap(self, name, channel, arch, download_path,
                       download_url, expected_sha512):
        if self._is_downloaded(download_path, expected_sha512):
            logger.info('Already downloaded {} at {}'.format(
                name, download_path))
            return
        logger.info('Downloading {}'.format(name, download_path))
        request = self.cpi.get(download_url, stream=True)
        request.raise_for_status()
        download_requests_stream(request, download_path)

        if self._is_downloaded(download_path, expected_sha512):
            logger.info('Successfully downloaded {} at {}'.format(
                name, download_path))
        else:
            raise errors.SHAMismatchError(download_path, expected_sha512)

    def _is_downloaded(self, path, expected_sha512):
        if not os.path.exists(path):
            return False

        file_sum = hashlib.sha512()
        with open(path, 'rb') as f:
            for file_chunk in iter(
                    lambda: f.read(file_sum.block_size * 128), b''):
                file_sum.update(file_chunk)
        return expected_sha512 == file_sum.hexdigest()

    def push_validation(self, snap_id, assertion):
        return self.sca.push_validation(snap_id, assertion)

    def get_validations(self, snap_id):
        return self.sca.get_validations(snap_id)

    def sign_developer_agreement(self, latest_tos_accepted=False):
        return self.sca.sign_developer_agreement(latest_tos_accepted)


class SSOClient(Client):
    """The Single Sign On server deals with authentication.

    It is used directly or indirectly by other servers.

    """
    def __init__(self, conf):
        super().__init__(conf, os.environ.get(
            'UBUNTU_SSO_API_ROOT_URL',
            constants.UBUNTU_SSO_API_ROOT_URL))

    def get_unbound_discharge(self, email, password, one_time_password,
                              caveat_id):
        data = dict(email=email, password=password,
                    caveat_id=caveat_id)
        if one_time_password:
            data['otp'] = one_time_password
        response = self.post(
            'tokens/discharge', data=json.dumps(data),
            headers={'Content-Type': 'application/json',
                     'Accept': 'application/json'})
        try:
            response_json = response.json()
        except JSONDecodeError:
            response_json = {}
        if response.ok:
            return response_json['discharge_macaroon']
        else:
            if (response.status_code == requests.codes.unauthorized and
                any(error.get('code') == 'twofactor-required'
                    for error in response_json.get('error_list', []))):
                raise errors.StoreTwoFactorAuthenticationRequired()
            else:
                raise errors.StoreAuthenticationError(
                    'Failed to get unbound discharge: {}'.format(
                        response.text))

    def refresh_unbound_discharge(self, unbound_discharge):
        data = {'discharge_macaroon': unbound_discharge}
        response = self.post(
            'tokens/refresh', data=json.dumps(data),
            headers={'Content-Type': 'application/json',
                     'Accept': 'application/json'})
        if response.ok:
            return response.json()['discharge_macaroon']
        else:
            raise errors.StoreAuthenticationError(
                'Failed to refresh unbound discharge: {}'.format(
                    response.text))


class SnapIndexClient(Client):
    """The Click Package Index knows everything about existing snaps.

    https://wiki.ubuntu.com/AppStore/Interfaces/ClickPackageIndex is the
    canonical reference.
    """

    def __init__(self, conf):
        super().__init__(conf, os.environ.get(
            'UBUNTU_STORE_SEARCH_ROOT_URL',
            constants.UBUNTU_STORE_SEARCH_ROOT_URL))

    def get_package(self, snap_name, channel, arch=None):
        headers = {
            'Accept': 'application/hal+json',
            'X-Ubuntu-Release': constants.DEFAULT_SERIES,
        }
        if arch:
            headers['X-Ubuntu-Architecture'] = arch

        params = {
            'channel': channel,
            'fields': 'status,anon_download_url,download_url,'
                      'download_sha512,snap_id,release',
        }
        logger.info('Getting details for {}'.format(snap_name))
        url = 'api/v1/snaps/details/{}'.format(snap_name)
        resp = self.get(url, headers=headers, params=params)
        if resp.status_code != 200:
            raise errors.SnapNotFoundError(snap_name, channel, arch)
        return resp.json()

    def get(self, url, headers=None, params=None, stream=False):
        if headers is None:
            headers = {}
        with contextlib.suppress(errors.InvalidCredentialsError):
            headers.update({'Authorization': _macaroon_auth(self.conf)})
        response = self.request('GET', url, stream=stream,
                                headers=headers, params=params)
        return response


class UpDownClient(Client):
    """The Up/Down server provide upload/download snap capabilities."""

    def __init__(self, conf):
        super().__init__(conf, os.environ.get(
            'UBUNTU_STORE_UPLOAD_ROOT_URL',
            constants.UBUNTU_STORE_UPLOAD_ROOT_URL))

    def upload(self, monitor):
        return self.post(
            urllib.parse.urljoin(self.root_url, 'unscanned-upload/'),
            data=monitor, headers={'Content-Type': monitor.content_type,
                                   'Accept': 'application/json'})


class SCAClient(Client):
    """The software center agent deals with managing snaps."""

    def __init__(self, conf):
        super().__init__(conf, os.environ.get(
            'UBUNTU_STORE_API_ROOT_URL',
            constants.UBUNTU_STORE_API_ROOT_URL))

    def get_macaroon(self, acls, packages=None, channels=None):
        data = {
            'permissions': acls,
        }
        if packages is not None:
            data.update({
                'packages': packages,
            })
        if channels is not None:
            data.update({
                'channels': channels,
            })
        headers = {
            'Accept': 'application/json',
        }
        response = self.post(
            'acl/', json=data, headers=headers)
        if response.ok:
            return response.json()['macaroon']
        else:
            raise errors.StoreAuthenticationError('Failed to get macaroon')

    @staticmethod
    def _is_needs_refresh_response(response):
        return (
            response.status_code == requests.codes.unauthorized and
            response.headers.get('WWW-Authenticate') == (
                'Macaroon needs_refresh=1'))

    def request(self, *args, **kwargs):
        response = super().request(*args, **kwargs)
        if self._is_needs_refresh_response(response):
            raise errors.StoreMacaroonNeedsRefreshError()
        return response

    def get_account_information(self):
        auth = _macaroon_auth(self.conf)
        response = self.get(
            'account',
            headers={'Authorization': auth,
                     'Accept': 'application/json'})
        if response.ok:
            return response.json()
        else:
            raise errors.StoreAccountInformationError(response)

    def register_key(self, account_key_request):
        data = {'account_key_request': account_key_request}
        auth = _macaroon_auth(self.conf)
        response = self.post(
            'account/account-key', data=json.dumps(data),
            headers={'Authorization': auth,
                     'Content-Type': 'application/json',
                     'Accept': 'application/json'})
        if not response.ok:
            raise errors.StoreKeyRegistrationError(response)

    def register(self, snap_name, is_private, series):
        auth = _macaroon_auth(self.conf)
        data = dict(snap_name=snap_name, is_private=is_private,
                    series=series)
        response = self.post(
            'register-name/', data=json.dumps(data),
            headers={'Authorization': auth,
                     'Content-Type': 'application/json'})
        if not response.ok:
            raise errors.StoreRegistrationError(snap_name, response)

    def snap_push_metadata(self, snap_name, updown_data):
        data = {
            'name': snap_name,
            'series': constants.DEFAULT_SERIES,
            'updown_id': updown_data['upload_id'],
            'binary_filesize': updown_data['binary_filesize'],
            'source_uploaded': updown_data['source_uploaded'],
        }
        auth = _macaroon_auth(self.conf)
        response = self.post(
            'snap-push/', data=json.dumps(data),
            headers={'Authorization': auth,
                     'Content-Type': 'application/json',
                     'Accept': 'application/json'})
        if not response.ok:
            raise errors.StorePushError(data['name'], response)

        return StatusTracker(response.json()['status_details_url'])

    def snap_release(self, snap_name, revision, channels):
        data = {
            'name': snap_name,
            'revision': str(revision),
            'channels': channels,
        }
        auth = _macaroon_auth(self.conf)
        response = self.post(
            'snap-release/', data=json.dumps(data),
            headers={'Authorization': auth,
                     'Content-Type': 'application/json',
                     'Accept': 'application/json'})
        if not response.ok:
            raise errors.StoreReleaseError(data['name'], response)

        response_json = response.json()

        return response_json

    def push_validation(self, snap_id, assertion):
        data = {
            'assertion': assertion.decode('utf-8'),
        }
        auth = _macaroon_auth(self.conf)
        response = self.put(
            'snaps/{}/validations'.format(snap_id), data=json.dumps(data),
            headers={'Authorization': auth,
                     'Content-Type': 'application/json',
                     'Accept': 'application/json'})
        if not response.ok:
            raise errors.StoreValidationError(snap_id, response)
        try:
            response_json = response.json()
        except JSONDecodeError:
            message = ('Invalid response from the server when pushing '
                       'validations: {} {}').format(
                           response.status_code, response)
            logger.debug(message)
            raise errors.StoreValidationError(
                snap_id, response, message='Invalid response from the server')

        return response_json

    def get_validations(self, snap_id):
        auth = _macaroon_auth(self.conf)
        response = self.get(
            'snaps/{}/validations'.format(snap_id),
            headers={'Authorization': auth,
                     'Content-Type': 'application/json',
                     'Accept': 'application/json'})
        if not response.ok:
            raise errors.StoreValidationError(snap_id, response)
        try:
            response_json = response.json()
        except JSONDecodeError:
            message = ('Invalid response from the server when getting '
                       'validations: {} {}').format(
                           response.status_code, response)
            logger.debug(message)
            raise errors.StoreValidationError(
                snap_id, response, message='Invalid response from the server')

        return response_json

    def push_snap_build(self, snap_id, snap_build):
        url = 'snaps/{}/builds'.format(snap_id)
        data = json.dumps({"assertion": snap_build})
        headers = {
            'Authorization': _macaroon_auth(self.conf),
            'Content-Type': 'application/json'
        }
        response = self.post(url, data=data, headers=headers)
        if not response.ok:
            raise errors.StoreSnapBuildError(response)

    def snap_history(self, snap_id, series, arch):
        qs = {}
        if series:
            qs['series'] = series
        if arch:
            qs['arch'] = arch
        url = 'snaps/' + snap_id + '/history'
        if qs:
            url += '?' + urllib.parse.urlencode(qs)
        auth = _macaroon_auth(self.conf)
        response = self.get(
            url,
            headers={'Authorization': auth,
                     'Content-Type': 'application/json',
                     'Accept': 'application/json'})
        if not response.ok:
            raise errors.StoreSnapHistoryError(response, snap_id, series, arch)

        response_json = response.json()

        return response_json

    def snap_status(self, snap_id, series, arch):
        qs = {}
        if series:
            qs['series'] = series
        if arch:
            qs['arch'] = arch
        url = 'snaps/' + snap_id + '/status'
        if qs:
            url += '?' + urllib.parse.urlencode(qs)
        auth = _macaroon_auth(self.conf)
        response = self.get(
            url,
            headers={'Authorization': auth,
                     'Content-Type': 'application/json',
                     'Accept': 'application/json'})
        if not response.ok:
            raise errors.StoreSnapStatusError(response, snap_id, series, arch)

        response_json = response.json()

        return response_json

    def close_channels(self, snap_id, channel_names):
        url = 'snaps/{}/close'.format(snap_id)
        data = {
            'channels': channel_names
        }
        headers = {
            'Authorization': _macaroon_auth(self.conf),
        }
        response = self.post(url, json=data, headers=headers)
        if not response.ok:
            raise errors.StoreChannelClosingError(response)

        try:
            results = response.json()
            return results['closed_channels'], results['channel_maps']
        except (JSONDecodeError, KeyError):
            logger.debug(
                'Invalid response from the server on channel closing:\n'
                '{} {}\n{}'.format(response.status_code, response.reason,
                                   response.content))
            raise errors.StoreChannelClosingError(response)

    def sign_developer_agreement(self, latest_tos_accepted=False):
        auth = _macaroon_auth(self.conf)
        data = {'latest_tos_accepted': latest_tos_accepted}
        response = self.post(
            'agreement/', data=json.dumps(data),
            headers={'Authorization': auth,
                     'Content-Type': 'application/json',
                     'Accept': 'application/json'})

        if not response.ok:
            raise errors.DeveloperAgreementSignError(response)
        return response.json()


class StatusTracker:

    __messages = {
        'being_processed': 'Processing...',
        'ready_to_release': 'Ready to release!',
        'need_manual_review': 'Will need manual review...',
        'processing_error': 'Error while processing...',
    }

    __error_codes = (
        'processing_error',
        'need_manual_review',
    )

    def __init__(self, status_details_url):
        self.__status_details_url = status_details_url

    def track(self):
        queue = Queue()
        thread = Thread(target=self._update_status, args=(queue,))
        thread.start()

        widgets = ['Processing...', AnimatedMarker()]
        progress_indicator = ProgressBar(widgets=widgets, maxval=UnknownLength)
        progress_indicator.start()

        content = {}
        for indicator_count in itertools.count():
            if not queue.empty():
                content = queue.get()
                if isinstance(content, Exception):
                    raise content
                widgets[0] = self._get_message(content)
            progress_indicator.update(indicator_count)
            if content.get('processed'):
                break
            sleep(0.1)
        progress_indicator.finish()

        self.__content = content

        return content

    def raise_for_code(self):
        if any(self.__content['code'] == k for k in self.__error_codes):
            raise errors.StoreReviewError(self.__content)

    def _get_message(self, content):
        return self.__messages.get(content['code'], content['code'])

    def _update_status(self, queue):
        for content in self._get_status():
            queue.put(content)
            if content['processed']:
                break
            sleep(constants.SCAN_STATUS_POLL_DELAY)

    def _get_status(self):
        connection_errors_allowed = 10
        while True:
            try:
                content = requests.get(self.__status_details_url).json()
            except (requests.ConnectionError, requests.HTTPError) as e:
                if not connection_errors_allowed:
                    yield e
                content = {'processed': False, 'code': 'being_processed'}
                connection_errors_allowed -= 1
            yield content
