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
from unittest.mock import Mock, patch

import responses

from snapcraft import tests
from snapcraft.storeapi.common import store_api_call


class ApiCallTestCase(tests.TestCase):

    def setUp(self):
        super(ApiCallTestCase, self).setUp()
        p = patch('snapcraft.storeapi.common.os')
        mock_os = p.start()
        self.addCleanup(p.stop)
        mock_os.environ = {'UBUNTU_STORE_API_ROOT_URL': 'http://example.com'}

    @responses.activate
    def test_get_success(self):
        response_data = {'response': 'value'}
        responses.add(responses.GET, 'http://example.com/path',
                      body=json.dumps(response_data))

        result = store_api_call('/path')
        self.assertEqual(result, {
            'success': True,
            'data': response_data,
            'errors': [],
        })

    @responses.activate
    def test_get_error(self):
        response_data = {'response': 'error'}
        responses.add(responses.GET, 'http://example.com/path',
                      body=json.dumps(response_data), status=500)

        result = store_api_call('/path')
        self.assertEqual(result, {
            'success': False,
            'data': None,
            'errors': [json.dumps(response_data)],
        })

    @responses.activate
    def test_post_success(self):
        response_data = {'response': 'value'}
        responses.add(responses.POST, 'http://example.com/path',
                      body=json.dumps(response_data))

        result = store_api_call('/path', method='POST')
        self.assertEqual(result, {
            'success': True,
            'data': response_data,
            'errors': [],
        })

    @responses.activate
    def test_post_error(self):
        response_data = {'response': 'value'}
        responses.add(responses.POST, 'http://example.com/path',
                      body=json.dumps(response_data), status=500)

        result = store_api_call('/path', method='POST')
        self.assertEqual(result, {
            'success': False,
            'data': None,
            'errors': [json.dumps(response_data)],
        })

    def test_unsupported_method(self):
        self.assertRaises(ValueError, store_api_call, '/path', method='FOO')

    def test_get_with_session(self):
        session = Mock()
        store_api_call('/path', session=session)
        session.get.assert_called_once_with('http://example.com/path')

    def test_post_with_session(self):
        session = Mock()
        store_api_call('/path', method='POST', session=session)
        session.post.assert_called_once_with(
            'http://example.com/path',
            data=None, headers={'Content-Type': 'application/json'})

    @responses.activate
    def test_post_with_data(self):
        response_data = {'response': 'value'}
        responses.add(responses.POST, 'http://example.com/path',
                      body=json.dumps(response_data))

        result = store_api_call(
            '/path', method='POST', data={'request': 'value'})
        self.assertEqual(result, {
            'success': True,
            'data': response_data,
            'errors': [],
        })
        self.assertEqual(len(responses.calls), 1)
        self.assertEqual(responses.calls[0].request.headers['Content-Type'],
                         'application/json')
        self.assertEqual(responses.calls[0].request.body,
                         json.dumps({'request': 'value'}))
