# -*- coding: utf-8 -*-
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
from __future__ import absolute_import, unicode_literals
import os

from ssoclient.v2 import (
    ApiException,
    UnexpectedApiError,
    V2ApiClient,
)

from .common import (
    store_api_call,
)
from .constants import (
    UBUNTU_SSO_API_ROOT_URL,
)


def login(email, password, token_name='unused', otp=''):
    """Log in via the Ubuntu One SSO API.

    If successful, returns the root and discharge macaroons.
    """
    result = {
        'success': False,
        'body': None,
    }

    api_endpoint = os.environ.get(
        'UBUNTU_SSO_API_ROOT_URL', UBUNTU_SSO_API_ROOT_URL)
    client = V2ApiClient(endpoint=api_endpoint)

    macaroon, error = get_package_upload_macaroon()
    if macaroon is None:
        result['errors'] = error
    else:
        result['success'] = True
    if result['success']:
        data = {'email': email, 'password': password, 'macaroon': macaroon}
        if otp:
            data['otp'] = otp
        discharge, error = get_discharge_macaroon(client, data)
        if discharge is None:
            result['errors'] = error
            result['success'] = False
        else:
            result['body'] = dict(package_upload=(macaroon, discharge))
    return result


def get_package_upload_macaroon():
    response = store_api_call('../../api/2.0/acl/package_upload/',
                              method='POST', data={})
    if response['success']:
        return response['data']['macaroon'], None
    else:
        return None, response['errors']


def get_discharge_macaroon(client, data):
    try:
        response = client.session.post(
            '/tokens/discharge', data=data,
            headers={'Content-Type': 'application/json',
                     'Accept': 'application/json'})
        return response.content['discharge_macaroon'], None
    except ApiException as err:
        return None, err.body
    except UnexpectedApiError as err:
        return None, err.json_body
