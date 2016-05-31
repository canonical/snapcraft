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

import ssoclient.v2 as sso
import yaml

import snapcraft
from snapcraft import config
from snapcraft.storeapi import (
    common,
    constants,
    _upload,
)


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


class Client():
    """A base class to define clients for the ols servers.

    This is a simple wrapper around requests.Session so we inherit all good
    bits while providing a simple point for tests to override when needed.

    """

    def __init__(self, conf, root_url):
        self.conf = config
        self.root_url = root_url


class StoreClient():
    """High-level client for the V2.0 API SCA resources."""

    def __init__(self):
        super().__init__()
        self.conf = config.Config()
        self.cpi = SnapIndexClient(self.conf)
        self.updown = UpDownClient(self.conf)
        self.sca = SCAClient(self.conf)

    def login(self, email, password, token_name, one_time_password=None):
        """Log in via the Ubuntu One SSO API.

        If successful, returns the oauth token data.
        """
        result = {
            'success': False,
            'body': None,
        }

        api_endpoint = os.environ.get(
            'UBUNTU_SSO_API_ROOT_URL', constants.UBUNTU_SSO_API_ROOT_URL)
        client = sso.V2ApiClient(endpoint=api_endpoint)
        data = {
            'email': email,
            'password': password,
            'token_name': token_name,
        }
        if one_time_password:
            data['otp'] = one_time_password
        try:
            response = client.login(data=data)
            result['body'] = response
            result['success'] = True
        except sso.ApiException as err:
            result['body'] = err.body
        except sso.UnexpectedApiError as err:
            result['body'] = err.json_body

        if result['success']:
            for key, value in result['body'].items():
                self.conf.set(key, str(value))
            self.conf.save()

        return result

    def logout(self):
        self.conf.clear()
        self.conf.save()

    def upload(self, snap_filename):
        if not os.path.exists(snap_filename):
            raise FileNotFoundError(snap_filename)
        snap_name = self._get_name_from_snap_file(snap_filename)

        session = common.get_oauth_session(config.Config())
        if session is None:
            raise InvalidCredentialsError()
        data = _upload.upload_files(snap_filename, self.updown)
        success = data.get('success', False)
        if not success:
            return data

        result = _upload.upload_app(self.sca, snap_name, data)
        return result

    def _get_name_from_snap_file(self, snap_path):
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

    def download(self, snap_name, channel, download_path, arch=None):
        if arch is None:
            arch = snapcraft.ProjectOptions().deb_arch

        package = self.cpi.search_package(snap_name, channel, arch)
        if package is None:
            raise SnapNotFoundError(snap_name, channel, arch)
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
        download = self.cpi.get(download_url)
        with open(download_path, 'wb') as f:
            # FIXME: Cough, we may want to buffer here (and a progress bar
            # would be nice) -- vila 2016-04-26
            f.write(download.content)
        if self._is_downloaded(download_path, expected_sha512):
            logger.info('Successfully downloaded {} at {}'.format(
                name, download_path))
        else:
            raise SHAMismatchError(download_path, expected_sha512)

    def _is_downloaded(self, path, expected_sha512):
        if not os.path.exists(path):
            return False

        file_sum = hashlib.sha512()
        with open(path, 'rb') as f:
            for file_chunk in iter(
                    lambda: f.read(file_sum.block_size * 128), b''):
                file_sum.update(file_chunk)
        return expected_sha512 == file_sum.hexdigest()


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
        session = common.get_oauth_session(config.Config())
        if session is None:
            raise InvalidCredentialsError()

        session.headers.update({
            'accept': 'application/hal+json',
            'X-Ubuntu-Architecture': arch,
            'X-Ubuntu-Release': constants.DEFAULT_SERIES,
            'X-Ubuntu-Device-Channel': channel,
        })
        session.params = {
            'q': 'package_name:"{}"'.format(snap_name),
            'fields': 'download_url,anon_download_url,download_sha512',
            'size': 1,
        }
        logger.info('Getting details for {}'.format(snap_name))
        search = session.get(
            urllib.parse.urljoin(
                self.root_url, 'api/v1/search')).content.decode('utf-8')
        search_results = json.loads(search)
        logger.debug('search results {!r}'.format(search_results))

        embedded = search_results.get('_embedded', None)
        if embedded is None:
            return None
        else:
            return embedded['clickindex:package'][0]

    def get(self, download_url):
        session = common.get_oauth_session(config.Config())
        return session.get(download_url)


class UpDownClient(Client):
    """The Up/Down server provide upload/download snap capabilities."""

    def __init__(self, conf):
        super().__init__(conf, os.environ.get(
            'UBUNTU_STORE_UPLOAD_ROOT_URL',
            constants.UBUNTU_STORE_UPLOAD_ROOT_URL))

    def upload(self, monitor):
        session = common.get_oauth_session(config.Config())
        if session is None:
            raise InvalidCredentialsError()
        return session.post(
            urllib.parse.urljoin(self.root_url, 'unscanned-upload/'),
            data=monitor, headers={'Content-Type': monitor.content_type})


class SCAClient(Client):
    """The software center agent deals with managing snaps."""

    def __init__(self, conf):
        super().__init__(conf, os.environ.get(
            'UBUNTU_STORE_API_ROOT_URL',
            constants.UBUNTU_STORE_API_ROOT_URL))

    def snap_upload(self, data):
        session = common.get_oauth_session(config.Config())
        if session is None:
            raise InvalidCredentialsError()
        url = urllib.parse.urljoin(
            self.root_url, 'click-package-upload/{}/'.format(
                urllib.parse.quote_plus(data['name'])))
        return session.post(url, data)

    def get(self, download_url):
        session = common.get_oauth_session(config.Config())
        return session.get(download_url)
