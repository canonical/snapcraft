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
from .constants import (
    UBUNTU_SSO_API_ROOT_URL,
    UBUNTU_STORE_API_ROOT_URL,
)
from .info import get_info  # noqa
from ._download import download  # noqa
from ._register import register_name  # noqa
from ._upload import upload  # noqa


class V2ApiClient(object):
    """High-level client for the V2.0 API SCA resources."""

    def __init__(self, sso_class=None):
        if sso_class is None:
            sso_class = sso.V2ApiClient
        self.with_macaroons = os.environ.get('SNAPCRAFT_WITH_MACAROONS', False)
        self.conf = config.Config()
        self.session = requests.Session()
        self.root_url = os.environ.get('UBUNTU_STORE_API_ROOT_URL',
                                       UBUNTU_STORE_API_ROOT_URL)
        sso_endpoint = os.environ.get(
            'UBUNTU_SSO_API_ROOT_URL', UBUNTU_SSO_API_ROOT_URL)
        self.sso = sso_class(sso_endpoint)

    def login(self, email, password, one_time_password=None):
        if self.with_macaroons:
            return self.macaroon_login(email, password, one_time_password)
        else:
            return self.oauth_login(email, password, one_time_password)

    def logout(self):
        self.conf.clear()
        self.conf.save()

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
        macaroon, error = self.get_macaroon('package_upload')
        if macaroon is None:
            result['errors'] = error
        else:
            result['success'] = True
        if result['success']:
            discharge, error = self.get_discharge(email, password,
                                                  one_time_password, macaroon)
            if discharge is None:
                result['errors'] = error
                result['success'] = False
            else:
                conf = config.Config()
                conf.set('package_upload', ','.join([macaroon, discharge]))
                conf.save()
        return result

    def get_macaroon(self, acl):
        response = self.post('../../api/2.0/acl/{}/'.format(acl), data={})
        if response['success']:
            return response['data']['macaroon'], None
        else:
            return None, response['errors']

    def get_discharge(self, email, password, one_time_password, macaroon):
        data = dict(email=email, password=password, macaroon=macaroon)
        if one_time_password:
            data['otp'] = one_time_password
        try:
            response = self.sso.post(
                '/tokens/discharge', data=data,
                headers={'Content-Type': 'application/json',
                         'Accept': 'application/json'})
            return response.content['discharge_macaroon'], None
        except sso.ApiException as err:
            return None, err.body
        except sso.UnexpectedApiError as err:
            return None, err.json_body

    def post(self, path, data, headers=None):
        if headers is None:
            headers = {}
        if data is not None:
            data = json.dumps(data)
        headers.update({'Content-Type': 'application/json'})
        url = parse.urljoin(self.root_url, path)
        response = self.session.post(url, data=data, headers=headers)
        return response
