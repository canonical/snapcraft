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
import subprocess
import tempfile
import urllib.parse

import requests
import yaml

import snapcraft
from snapcraft import config
from snapcraft.storeapi import (
    _upload,
    constants,
    errors,
    macaroons
)


logger = logging.getLogger(__name__)


def _get_name_from_snap_file(snap_path):
    with tempfile.TemporaryDirectory() as temp_dir:
        output = subprocess.check_output(
            ['unsquashfs', '-d',
             os.path.join(temp_dir, 'squashfs-root'),
             snap_path, '-e', os.path.join('meta', 'snap.yaml')])
        logger.debug(output)
        with open(os.path.join(
                temp_dir, 'squashfs-root', 'meta', 'snap.yaml')
        ) as yaml_file:
            snap_yaml = yaml.load(yaml_file)

    return snap_yaml['name']


def _macaroon_auth(conf):
    """Format a macaroon and its associated discharge.

    :return: A string suitable to use in an Authorization header.

    """
    root_macaroon_raw = conf.get('macaroon')
    unbound_raw = conf.get('unbound_discharge')
    root_macaroon = _deserialize_macaroon(root_macaroon_raw)
    unbound = _deserialize_macaroon(unbound_raw)
    bound = root_macaroon.prepare_for_request(unbound)
    discharge_macaroon_raw = bound.serialize()
    auth = 'Macaroon root={}, discharge={}'.format(
        root_macaroon_raw, discharge_macaroon_raw)
    return auth


def _deserialize_macaroon(value):
    try:
        return macaroons.Macaroon.deserialize(value)
    except:
        raise errors.InvalidCredentialsError()


class Client():
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
        final_url = urllib.parse.urljoin(self.root_url, url)
        response = self.session.request(
            method, final_url, headers=headers,
            params=params, **kwargs)
        return response

    def get(self, url, **kwargs):
        return self.request('GET', url, **kwargs)

    def post(self, url, **kwargs):
        return self.request('POST', url, **kwargs)


class StoreClient():
    """High-level client for the V2.0 API SCA resources."""

    def __init__(self):
        super().__init__()
        self.conf = config.Config()
        self.sso = SSOClient(self.conf)
        self.cpi = SnapIndexClient(self.conf)
        self.updown = UpDownClient(self.conf)
        self.sca = SCAClient(self.conf)

    def login(self, email, password, one_time_password=None):
        """Log in via the Ubuntu One SSO API."""
        result = dict(success=False, body=None)
        # Ask the store for the needed capabalities to be associated with the
        # macaroon.
        macaroon, error = self.sca.get_macaroon(
            ['package_upload', 'package_access'])
        if error:
            result['errors'] = error
        else:
            caveat_id = self._extract_caveat_id(macaroon)
            if caveat_id is None:
                error = 'Invalid macaroon'
            else:
                unbound_discharge, error = self.sso.get_unbound_discharge(
                    email, password, one_time_password, caveat_id)
            if error:
                result['errors'] = error
            else:
                # The macaroon has been discharged, save it in the config
                self.conf.set('macaroon', macaroon)
                self.conf.set('unbound_discharge', unbound_discharge)
                self.conf.save()
                result['success'] = True
        return result

    def _extract_caveat_id(self, root_macaroon):
        macaroon = macaroons.Macaroon.deserialize(root_macaroon)
        # macaroons are all bytes, never strings
        sso_host = macaroons.convert_to_bytes(
            urllib.parse.urlparse(self.sso.root_url).hostname)
        for caveat in macaroon.caveats:
            if caveat.location == sso_host:
                return macaroons.convert_to_string(caveat.caveat_id)
        return None

    def logout(self):
        self.conf.clear()
        self.conf.save()

    def upload(self, snap_filename):
        if not os.path.exists(snap_filename):
            raise FileNotFoundError(snap_filename)
        snap_name = _get_name_from_snap_file(snap_filename)

        if self.conf.get('unbound_discharge') is None:
            raise errors.InvalidCredentialsError()
        data = _upload.upload_files(snap_filename, self.updown)
        success = data.get('success', False)
        if not success:
            return data

        result = _upload.upload_app(self.sca, snap_name, data)
        return result

    def download(self, snap_name, channel, download_path, arch=None):
        if arch is None:
            arch = snapcraft.ProjectOptions().deb_arch

        package = self.cpi.search_package(snap_name, channel, arch)
        if package is None:
            raise errors.SnapNotFoundError(snap_name, channel, arch)
        self._download_snap(
            snap_name, channel, arch, download_path,
            package['download_url'], package['download_sha512'])

    def _download_snap(self, name, channel, arch, download_path,
                       download_url, expected_sha512):
        if self._is_downloaded(download_path, expected_sha512):
            logger.info('Already downloaded {} at {}'.format(
                name, download_path))
            return
        logger.info('Downloading {}'.format(name, download_path))
        # FIXME: Check the status code ! -- vila 2016-05-04
        download = self.cpi.get(download_url)
        with open(download_path, 'wb') as f:
            # FIXME: Cough, we may want to buffer here (and a progress bar
            # would be nice) -- vila 2016-04-26
            f.write(download.content)
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


class SSOClient(Client):
    """The Single Sign On server deals with authentification.

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
        if response.ok:
            return response.json()['discharge_macaroon'], None
        else:
            return None, response.text


class SnapIndexClient(Client):
    """The Click Package Index knows everything about existing snaps.

    https://wiki.ubuntu.com/AppStore/Interfaces/ClickPackageIndex is the
    canonical reference.
    """

    def __init__(self, conf):
        super().__init__(conf, os.environ.get(
            'UBUNTU_STORE_SEARCH_ROOT_URL',
            constants.UBUNTU_STORE_SEARCH_ROOT_URL))

    def search_package(self, snap_name, channel, arch):
        if self.conf.get('unbound_discharge') is None:
            raise errors.InvalidCredentialsError()

        headers = {
            'Accept': 'application/hal+json',
            'X-Ubuntu-Architecture': arch,
            'X-Ubuntu-Release': constants.DEFAULT_SERIES,
            'X-Ubuntu-Device-Channel': channel,
        }
        params = {
            'q': 'package_name:"{}"'.format(snap_name),
            'fields': 'status,download_url,download_sha512',
            'size': 1,
        }
        logger.info('Getting details for {}'.format(snap_name))
        resp = self.get('api/v1/search', headers=headers, params=params)
        embedded = resp.json().get('_embedded', None)
        if embedded is None:
            return None
        else:
            return embedded['clickindex:package'][0]

    def get(self, url, headers=None, params=None):
        if headers is None:
            headers = {}
        headers.update({'Authorization': _macaroon_auth(self.conf)})
        response = self.request('GET', url, headers=headers, params=params)
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

    def get_macaroon(self, acls):
        response = self.post(
            'acl/',
            data=json.dumps({'permissions': acls}),
            headers={'Content-Type': 'application/json',
                     'Accept': 'application/json'})
        if response.ok:
            return response.json()['macaroon'], None
        else:
            return None, response.text

    def snap_upload(self, data):
        auth = _macaroon_auth(self.conf)
        response = self.post(
            'snap-push/', data=json.dumps(data),
            headers={'Authorization': auth,
                     'Content-Type': 'application/json',
                     'Accept': 'application/json'})
        return response
