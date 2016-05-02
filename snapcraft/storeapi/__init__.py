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
import hashlib
import json
import logging
import os

from urllib import parse

import requests

import snapcraft
from snapcraft import config
from .constants import (
    DEFAULT_SERIES,
    UBUNTU_SSO_API_ROOT_URL,
    UBUNTU_STORE_API_ROOT_URL,
    UBUNTU_STORE_SEARCH_ROOT_URL,
)

from . import _upload
from .info import get_info  # noqa


logger = logging.getLogger(__name__)


class InvalidCredentialsError(Exception):
    pass


class StoreError(Exception):
    """Base class for all storeapi exceptions.

    :cvar fmt: A format string that daughter classes override
    """

    fmt = 'Daughter classes should redefine this'

    def __init__(self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)

    def __str__(self):
        return self.fmt.format([], **self.__dict__)


class SnapNotFoundError(StoreError):

    fmt = 'The "{name}" for {arch} was not found in {channel}.'

    def __init__(self, name, channel, arch):
        super().__init__(name=name, channel=channel, arch=arch)


class SHAMismatchError(StoreError):

    fmt = 'SHA512 checksum for {path} is not {expected_sha}.'

    def __init__(self, path, expected_sha):
        super().__init__(path=path, expected_sha=expected_sha)


def _macaroon_auth(conf, acl):
    """Format a macaroon and its associated discharge.

    :return: A string suitable to use in an Authorization header.
    """
    macaroon, discharge = conf.get_macaroon(acl)
    auth = 'Macaroon root={}, discharge={}'.format(macaroon, discharge)
    return auth


class Client(object):
    """A base class to define clients for the ols servers.

    This is a simple wrapper around requests.Session so we inherit all good
    bits while providing a simple point for tests to override when needed.
    """

    def __init__(self, conf, root_url):
        self.conf = conf
        self.root_url = root_url
        self.session = requests.Session()

    def request(self, method, url, params=None, headers=None, **kwargs):
        """Overriding base class to handle the root url."""
        # Note that url may be absolute in which case 'root_url' is ignored by
        # urljoin.
        final_url = parse.urljoin(self.root_url, url)
        response = self.session.request(method, final_url, headers=headers,
                                        params=params, **kwargs)
        return response

    def get(self, url, **kwargs):
        return self.request('GET', url, **kwargs)

    def post(self, url, **kwargs):
        return self.request('POST', url, **kwargs)

    def close(self):
        self.session.close()


class SCAClient(Client):
    """High-level client for the V2.0 API SCA resources."""

    def __init__(self):
        super().__init__(config.Config(),
                         os.environ.get('UBUNTU_STORE_API_ROOT_URL',
                                        UBUNTU_STORE_API_ROOT_URL))
        self.sso = SSOClient(self.conf)
        # Will be set by upload()
        self.updown = None
        # Will be set by download()
        self.cpi = None

    # API for snapcraft

    def login(self, email, password, one_time_password=None):
        result = dict(success=False, body=None)
        acls = ('store_read', 'store_write',
                'package_access', 'package_upload')
        macaroons = {}
        for acl in acls:
            macaroon, error = self.get_macaroon(acl)
            if error:
                result['errors'] = error
                break
            macaroons[acl] = macaroon
        else:
            discharges, error = self.get_discharges(
                email, password, one_time_password, macaroons)
            if error:
                result['errors'] = error
            else:
                # All macaroons have been discharged, save them in the config
                for acl in acls:
                    self.conf.set(acl, ','.join([macaroons[acl],
                                                 discharges[acl]]))
                self.conf.save()
                result['success'] = True
        return result

    def logout(self):
        self.conf.clear()
        self.conf.save()

    def register_name(self, name):
        data = dict(snap_name=name, series=DEFAULT_SERIES)
        auth = _macaroon_auth(self.conf, 'package_upload')
        response = self.post('register-name/',
                             data=json.dumps(data),
                             headers={'Authorization': auth,
                                      'Content-Type': 'application/json'})
        if not response.ok:
            # if (response['errors'] == ['Authorization Required']
            #     and (response.headers['WWW-Authenticate']
            #          == "Macaroon needs_refresh=1":
            # Refresh the discharge macaroon and retry
            pass
        return response

    def upload(self, binary_path, snap_name):
        if self.conf.get('package_upload') is None:
            raise InvalidCredentialsError()
        self.updown = requests.Session()
        data = _upload.upload_files(binary_path, self.updown)
        success = data.get('success', False)
        if not success:
            return data

        result = _upload.upload_app(self, snap_name, data)
        return result

    def download(self, snap_name, channel, download_path, arch=None):
        if arch is None:
            arch = snapcraft.ProjectOptions().deb_arch
        self.cpi = CPIClient(self.conf)
        logger.info('Getting details for {}'.format(snap_name))
        package = self.cpi.search_package(snap_name, channel, arch)
        if package is None:
            raise SnapNotFoundError(snap_name, channel, arch)
        return self.download_snap(snap_name, channel, arch,
                                  download_path, package['download_url'],
                                  package['download_sha512'])

    def get_macaroon(self, acl):
        response = self.post('../../api/2.0/acl/{}/'.format(acl),
                             data=json.dumps({}),
                             headers={'Content-Type': 'application/json'})
        if response.ok:
            return response.json()['macaroon'], None
        else:
            return None, response.text

    def get_discharges(self, email, password, one_time_password, macaroons):
        data = dict(email=email, password=password,
                    macaroons=[(k, v) for k, v in macaroons.items()])
        if one_time_password:
            data['otp'] = one_time_password
        response = self.sso.post(
            'tokens/discharge', data=json.dumps(data),
            headers={'Content-Type': 'application/json',
                     'Accept': 'application/json'})
        if response.ok:
            return dict(response.json()['discharge_macaroons']), None
        else:
            return None, response.text

    def upload_snap(self, name, data):
        data['name'] = name
        headers = {'Authorization':
                   _macaroon_auth(self.conf, 'package_upload')}
        response = self.post('snap-upload/', data=data, headers=headers)
        return response

    def download_snap(self, name, channel, arch, download_path,
                      download_url, expected_sha512):
        if self.matching_sha512(download_path, expected_sha512):
            logger.info('Already downloaded {} at {}'.format(
                name, download_path))
            return
        logger.info('Downloading {}'.format(name, download_path))
        download = self.cpi.get(download_url)
        with open(download_path, 'wb') as f:
            # FIXME: Cough, we may want to buffer here (and a progress bar
            # would be nice) -- vila 2016-04-26
            f.write(download.content)
        if self.matching_sha512(download_path, expected_sha512):
            logger.info('Successfully downloaded {} at {}'.format(
                name, download_path))
        else:
            raise SHAMismatchError(download_path, expected_sha512)

    def matching_sha512(self, path, expected_sha512):
        if not os.path.exists(path):
            return False

        file_sum = hashlib.sha512()
        with open(path, 'rb') as f:
            for file_chunk in iter(
                    lambda: f.read(file_sum.block_size * 128), b''):
                file_sum.update(file_chunk)
        return expected_sha512 == file_sum.hexdigest()

    def close(self):
        if self.cpi is not None:
            self.cpi.close()
        if self.updown is not None:
            self.updown.close()
        self.sso.close()
        super().close()


class SSOClient(Client):
    """The Single Sign On server deals with authentification.

    It is used directly or indirectly by other servers.
    """

    def __init__(self, conf):
        super().__init__(conf, os.environ.get('UBUNTU_SSO_API_ROOT_URL',
                                              UBUNTU_SSO_API_ROOT_URL))


class CPIClient(Client):
    """The Click Package Index knows everything about existing snaps.

    https://wiki.ubuntu.com/AppStore/Interfaces/ClickPackageIndex is the
    canonical reference.
    """

    def __init__(self, conf):
        super().__init__(conf, os.environ.get('UBUNTU_STORE_SEARCH_ROOT_URL',
                                              UBUNTU_STORE_SEARCH_ROOT_URL))
        if self.conf.get('package_access') is None:
            raise InvalidCredentialsError()

    def search_package(self, snap_name, channel, arch):
        headers = {
            'Accept': 'application/hal+json',
            'X-Ubuntu-Architecture': arch,
            'X-Ubuntu-Release': DEFAULT_SERIES,
            'X-Ubuntu-Device-Channel': channel,
        }
        params = {
            'q': 'package_name:"{}"'.format(snap_name),
            'fields': 'status,download_url,anon_download_url,download_sha512',
            'size': 1,
        }
        resp = self.get('api/v1/search', headers=headers, params=params)
        embedded = resp.json().get('_embedded', None)
        if embedded is None:
            return None
        else:
            return embedded['clickindex:package'][0]

    def get(self, url, headers=None, params=None):
        if headers is None:
            headers = {}
        headers.update({'Authorization':
                        _macaroon_auth(self.conf, 'package_access')})
        response = self.request('GET', url, headers=headers, params=params)
        return response
