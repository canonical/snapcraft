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

import os

import ssoclient.v2 as sso

from .channels import get_channels, update_channels  # noqa
from .constants import UBUNTU_SSO_API_ROOT_URL
from .info import get_info  # noqa
from ._download import download  # noqa
from ._upload import upload  # noqa
from snapcraft import config


class StoreClient():
    """High-level client for the V2.0 API SCA resources."""

    def __init__(self):
        super().__init__()
        self.conf = config.Config()

    def login(self, email, password, token_name, one_time_password=None):
        """Log in via the Ubuntu One SSO API.

        If successful, returns the oauth token data.
        """
        result = {
            'success': False,
            'body': None,
        }

        api_endpoint = os.environ.get(
            'UBUNTU_SSO_API_ROOT_URL', UBUNTU_SSO_API_ROOT_URL)
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
