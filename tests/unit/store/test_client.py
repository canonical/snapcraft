# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2018 Canonical Ltd
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

import requests
from requests import exceptions
from requests.packages import urllib3

from unittest import mock

from snapcraft.storeapi import _client
from snapcraft.storeapi import errors

from tests import unit


class ClientTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.client = _client.Client("conf", "root_url")

    @mock.patch.object(requests.Session, "request")
    def test_generic_network_error(self, mock_request):
        mock_request.side_effect = exceptions.ConnectionError("naughty error")
        self.assertRaises(
            errors.StoreNetworkError, self.client.request, "GET", "test-url"
        )

    @mock.patch.object(requests.Session, "request")
    def test_max_retries_error(self, mock_request):
        mock_request.side_effect = exceptions.ConnectionError(
            urllib3.exceptions.MaxRetryError(pool="test-pool", url="test-url")
        )
        self.assertRaises(
            errors.StoreNetworkError, self.client.request, "GET", "test-url"
        )
