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
import time
from functools import wraps

import requests
from requests_oauthlib import OAuth1Session

from .compat import urljoin
from .constants import UBUNTU_STORE_API_ROOT_URL


def get_oauth_session(conf):
    """Return a client configured to allow oauth signed requests."""
    consumer_key = conf.get('consumer_key')
    client_secret = conf.get('consumer_secret')
    resource_owner_key = conf.get('token_key')
    resource_owner_secret = conf.get('token_secret')
    if None in (consumer_key, client_secret,
                resource_owner_key, resource_owner_secret):
        return None

    session = OAuth1Session(
        consumer_key, client_secret=client_secret,
        resource_owner_key=resource_owner_key,
        resource_owner_secret=resource_owner_secret,
        signature_method='PLAINTEXT')
    return session


def get_macaroon_auth(config):
    # FIXME: Check config and errors if expected keys are not there asking to
    # 'login'. -- vila 2016-04-07
    auth = 'Macaroon root={}, discharge={}'.format(
        config['root_macaroon'], config['discharge_macaroon'])
    return auth


def store_raw_api_call(path, session=None, method='GET', data=None,
                       headers=None):
    if session is not None:
        client = session
    else:
        client = requests

    root_url = os.environ.get('UBUNTU_STORE_API_ROOT_URL',
                              UBUNTU_STORE_API_ROOT_URL)
    if headers is None:
        headers = {}
    url = urljoin(root_url, path)
    if method == 'GET':
        response = client.get(url, headers=headers)
    elif method == 'POST':
        if data is not None:
            data = json.dumps(data)
        headers.update({'Content-Type': 'application/json'})
        response = client.post(url, data=data,
                               headers=headers)
    else:
        raise ValueError('Method {} not supported'.format(method))
    return response


def store_api_call(path, session=None, method='GET', data=None):
    """Issue a request for a particular endpoint of the MyApps API."""
    result = {'success': False, 'errors': [], 'data': None}

    response = store_raw_api_call(path, session, method, data)

    if response.ok:
        result['success'] = True
        result['data'] = response.json()
    else:
        result['errors'] = [response.text]
    return result


def retry(terminator=None, retries=3, delay=3, backoff=2, logger=None):
    """Decorate a function to automatically retry calling it on failure.

    Arguments:
        - terminator: this should be a callable that returns a boolean;
          it is used to determine if the function call was successful
          and the retry loop should be stopped
        - retries: an integer specifying the maximum number of retries
        - delay: initial number of seconds to wait for the first retry
        - backoff: exponential factor to use to adapt the delay between
          subsequent retries
        - logger: logging.Logger instance to use for logging

    The decorated function will return as soon as any of the following
    conditions are met:

        1. terminator evaluates function output as True
        2. there are no more retries left

    If the terminator callable is not provided, the function will be called
    exactly once and will not be retried.

    """
    def decorated(func):
        if retries != int(retries) or retries < 0:
            raise ValueError(
                'retries value must be a positive integer or zero')
        if delay < 0:
            raise ValueError('delay value must be positive')

        if backoff != int(backoff) or backoff < 1:
            raise ValueError('backoff value must be a positive integer')

        @wraps(func)
        def wrapped(*args, **kwargs):
            return _wrapped_retry(
                terminator, retries, delay, backoff, logger,
                func, *args, **kwargs)

        return wrapped
    return decorated


def _wrapped_retry(terminator, retries, delay, backoff, logger,
                   func, *args, **kwargs):
    retries_left, current_delay = retries, delay

    result = func(*args, **kwargs)
    if terminator is not None:
        while not terminator(result) and retries_left > 0:
            msg = "... retrying in %d seconds" % current_delay
            if logger:
                logger.warning(msg)

            # sleep
            time.sleep(current_delay)
            retries_left -= 1
            current_delay *= backoff
            # retry
            result = func(*args, **kwargs)

    return result, retries_left == 0
