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

from snapcraft import config

from .constants import (
    UBUNTU_SSO_API_ROOT_URL,
)


def login(email, password, token_name, otp=''):
    """Log in via the Ubuntu One SSO API.

    If successful, returns the oauth token data.
    """
    result = {
        'success': False,
        'body': None,
    }

    api_endpoint = os.environ.get(
        'UBUNTU_SSO_API_ROOT_URL', UBUNTU_SSO_API_ROOT_URL)
    client = V2ApiClient(endpoint=api_endpoint)
    data = {
        'email': email,
        'password': password,
        'token_name': token_name,
    }
    if otp:
        data['otp'] = otp
    try:
        response = client.login(data=data)
        result['body'] = response
        result['success'] = True
        conf = config.Config()
        conf.load()
        for k in ('consumer_key', 'consumer_secret',
                  'token_key', 'token_secret'):
            conf.set(k, response[k])
        conf.save()
    except ApiException as err:
        result['body'] = err.body
    except UnexpectedApiError as err:
        result['body'] = err.json_body
    return result
