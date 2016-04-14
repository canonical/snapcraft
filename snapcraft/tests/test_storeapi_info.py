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
from unittest import TestCase

from unittest.mock import patch

from snapcraft.storeapi.info import get_info


class InfoAPITestCase(TestCase):

    def setUp(self):
        super(InfoAPITestCase, self).setUp()

        patcher = patch('snapcraft.storeapi.common.requests.get')
        self.mock_get = patcher.start()
        self.mock_response = self.mock_get.return_value
        self.addCleanup(patcher.stop)

    def test_get_info(self):
        expected = {
            'success': True,
            'errors': [],
            'data': {'version': 1},
        }
        self.mock_response.ok = True
        self.mock_response.json.return_value = {'version': 1}
        data = get_info()
        self.assertEqual(data, expected)

    def test_get_info_with_error_response(self):
        expected = {
            'success': False,
            'errors': ['some error'],
            'data': None,
        }
        self.mock_response.ok = False
        self.mock_response.text = 'some error'
        data = get_info()
        self.assertEqual(data, expected)

    def test_get_info_uses_environment_variables(self):
        with patch('snapcraft.storeapi.common.os.environ',
                   {'UBUNTU_STORE_API_ROOT_URL': 'http://example.com'}):
            get_info()
        self.mock_get.assert_called_once_with('http://example.com', headers={})
