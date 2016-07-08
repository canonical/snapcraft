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

import requests
from requests_oauthlib import OAuth1Session

from .compat import urljoin
from .constants import UBUNTU_STORE_API_ROOT_URL


def get_oauth_session(config):
    """Return a client configured to allow oauth signed requests."""
    consumer_key = config.get('consumer_key')
    consumer_secret = config.get('consumer_secret')
    token_key = config.get('token_key')
    token_secret = config.get('token_secret')
    if (consumer_key is not None and
            consumer_secret is not None and
            token_key is not None and
            token_secret is not None):
        return OAuth1Session(
            consumer_key,
            client_secret=consumer_secret,
            resource_owner_key=token_key,
            resource_owner_secret=token_secret,
            signature_method='PLAINTEXT',
        )


def store_api_call(path, session=None, method='GET', data=None):
    """Issue a request for a particular endpoint of the MyApps API."""
    result = {'success': False, 'errors': [], 'data': None}
    if session is not None:
        client = session
    else:
        client = requests

    root_url = os.environ.get('UBUNTU_STORE_API_ROOT_URL',
                              UBUNTU_STORE_API_ROOT_URL)
    url = urljoin(root_url, path)
    if method == 'GET':
        response = client.get(url)
    elif method == 'POST':
        response = client.post(url, data=data and json.dumps(data) or None,
                               headers={'Content-Type': 'application/json'})
    else:
        raise ValueError('Method {} not supported'.format(method))

    if response.ok:
        result['success'] = True
        result['data'] = response.json()
    else:
        result['errors'] = [response.text]
    return result
