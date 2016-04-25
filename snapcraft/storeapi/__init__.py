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
import json
import os

from urllib import parse

import requests
from ssoclient import v2 as sso

from snapcraft import config
from .channels import get_channels, update_channels  # noqa
from .common import get_oauth_session
from .constants import (
    UBUNTU_SSO_API_ROOT_URL,
    UBUNTU_STORE_API_ROOT_URL,
)

from . import _upload
from .info import get_info  # noqa
from ._download import download  # noqa


class InvalidCredentials(Exception):
    pass


class ScaClient(object):
    """High-level client for the V2.0 API SCA resources."""

    def __init__(self, sso_class=None):
        if sso_class is None:
            sso_class = sso.V2ApiClient
        self.conf = config.Config()
        self.session = requests.Session()
        self.root_url = os.environ.get('UBUNTU_STORE_API_ROOT_URL',
                                       UBUNTU_STORE_API_ROOT_URL)
        sso_url = os.environ.get(
            'UBUNTU_SSO_API_ROOT_URL', UBUNTU_SSO_API_ROOT_URL)
        self.sso = sso_class(sso_url)
        # Will be set by upload()
        self.updown = None

    # API for snapcraft

    def login(self, email, password, one_time_password=None):
        with_macaroons = os.environ.get('SNAPCRAFT_WITH_MACAROONS', False)
        if with_macaroons:
            return self.macaroon_login(email, password, one_time_password)
        else:
            return self.oauth_login(email, password, one_time_password)

    def logout(self):
        self.conf.clear()
        self.conf.save()

    def register_name(self, name):
        # snapcraft targets the '16' series, hardcode it until more choices
        # become available server side -- vila 2016-04-22
        data = dict(snap_name=name, series='16')
        macaroon_auth = self.get_macaroon_auth('package_upload')
        response = self.post('register-name/',
                             data=json.dumps(data),
                             headers={'Authorization': macaroon_auth,
                                      'Content-Type': 'application/json'})
        if not response.ok:
            # if (response['errors'] == ['Authorization Required']
            #     and (response.headers['WWW-Authenticate']
            #          == "Macaroon needs_refresh=1":
            # Refresh the discharge macaroon and retry
            pass
        return response

    def upload(self, binary_path, snap_name):
        with_macaroons = os.environ.get('SNAPCRAFT_WITH_MACAROONS', False)
        if with_macaroons:
            if self.conf.get('package_upload') is None:
                raise InvalidCredentials()
        else:
            self.session = get_oauth_session(self.conf)
            if self.session is None:
                raise InvalidCredentials()
        self.updown = requests.Session()
        data = _upload.upload_files(binary_path, self.updown)
        success = data.get('success', False)
        if not success:
            return data

        result = _upload.upload_app(self, snap_name, data)
        return result

    def oauth_login(self, email, password, one_time_password):
        data = dict(email=email, password=password, token_name='snapcraft')
        result = dict(success=False, body=None)
        if one_time_password:
            data['otp'] = one_time_password
        try:
            response = self.sso.login(data=data)
            result['body'] = response
            result['success'] = True
            # Save the credentials for later reuse
            for k in ('consumer_key', 'consumer_secret',
                      'token_key', 'token_secret'):
                self.conf.set(k, response[k])
            self.conf.save()
        except sso.ApiException as err:
            result['body'] = err.body
        except sso.UnexpectedApiError as err:
            result['body'] = err.json_body
        return result

    def macaroon_login(self, email, password, one_time_password):
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

    def get_macaroon(self, acl):
        response = self.post('../../api/2.0/acl/{}/'.format(acl),
                             data=json.dumps({}),
                             headers={'Content-Type': 'application/json'})
        if response.ok:
            return response.json()['macaroon'], None
        else:
            return None, response.text

    def get_macaroon_auth(self, acl):
        macaroon, discharge = self.conf.get_macaroon(acl)
        macaroon_auth = 'Macaroon root={}, discharge={}'.format(
            macaroon, discharge)
        return macaroon_auth

    def get_discharges(self, email, password, one_time_password, macaroons):
        data = dict(email=email, password=password,
                    macaroons=[(k, v) for k, v in macaroons.items()])
        if one_time_password:
            data['otp'] = one_time_password
        try:
            response = self.sso.session.post(
                '/tokens/discharge', data=data,
                headers={'Content-Type': 'application/json',
                         'Accept': 'application/json'})
            return dict(response.content['discharge_macaroons']), None
        except sso.ApiException as err:
            return None, err.body
        except sso.UnexpectedApiError as err:
            return None, err.json_body

    def upload_snap(self, name, data):

        headers = {}
        with_macaroons = os.environ.get('SNAPCRAFT_WITH_MACAROONS', False)
        if with_macaroons:
            macaroon_auth = self.get_macaroon_auth('package_upload')
            del data['binary_filesize']
            del data['source_uploaded']
            headers = {'Authorization': macaroon_auth}
            upload_path = 'snap-upload/'
        else:
            upload_path = 'click-package-upload/{}/'.format(
                parse.quote_plus(name))

        response = self.post(upload_path, data=data, headers=headers)
        return response

    # Low level helpers

    def post(self, path, data, headers=None):
        if headers is None:
            headers = {}
        url = parse.urljoin(self.root_url, path)
        response = self.session.post(url, data=data, headers=headers)
        return response

    def close(self):
        if self.session is not None:
            self.session.close()
        if self.sso.session is not None:
            self.sso.session.close()
        if self.updown is not None:
            self.updown.close()
