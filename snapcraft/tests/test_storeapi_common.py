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
from unittest import TestCase
from unittest.mock import Mock, call, patch

import responses

from snapcraft.storeapi.common import (
    store_api_call,
    retry,
)


class ApiCallTestCase(TestCase):

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
        session.get.assert_called_once_with('http://example.com/path',
                                            headers={})

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

    @responses.activate
    def test_post_with_empty_data(self):
        response_data = {'response': 'value'}
        responses.add(responses.POST, 'http://example.com/path',
                      body=json.dumps(response_data))

        result = store_api_call(
            '/path', method='POST', data={})
        self.assertEqual(result, {
            'success': True,
            'data': response_data,
            'errors': [],
        })
        self.assertEqual(len(responses.calls), 1)
        self.assertEqual(responses.calls[0].request.headers['Content-Type'],
                         'application/json')
        self.assertEqual(responses.calls[0].request.body, '{}')


class RetryDecoratorTestCase(TestCase):

    def target(self, *args, **kwargs):
        return dict(args=args, kwargs=kwargs)

    def test_retry(self):
        result, aborted = retry()(self.target)()
        self.assertEqual(result, dict(args=(), kwargs={}))
        self.assertEqual(aborted, False)

    @patch('snapcraft.storeapi.common.time.sleep')
    def test_retry_small_backoff(self, mock_sleep):
        mock_terminator = Mock()
        mock_terminator.return_value = False

        delay = 0.001
        result, aborted = retry(mock_terminator, retries=2,
                                delay=delay)(self.target)()

        self.assertEqual(result, dict(args=(), kwargs={}))
        self.assertEqual(aborted, True)
        self.assertEqual(mock_terminator.call_count, 3)
        self.assertEqual(mock_sleep.mock_calls, [
            call(delay),
            call(delay * 2),
        ])

    def test_retry_abort(self):
        mock_terminator = Mock()
        mock_terminator.return_value = False
        mock_logger = Mock()

        result, aborted = retry(mock_terminator, delay=0.001, backoff=1,
                                logger=mock_logger)(self.target)()

        self.assertEqual(result, dict(args=(), kwargs={}))
        self.assertEqual(aborted, True)
        self.assertEqual(mock_terminator.call_count, 4)
        self.assertEqual(mock_logger.warning.call_count, 3)

    def test_retry_with_invalid_retries(self):
        for value in (0.1, -1):
            with self.assertRaises(ValueError) as ctx:
                retry(retries=value)(self.target)
            self.assertEqual(
                str(ctx.exception),
                'retries value must be a positive integer or zero')

    def test_retry_with_negative_delay(self):
        with self.assertRaises(ValueError) as ctx:
            retry(delay=-1)(self.target)
        self.assertEqual(str(ctx.exception),
                         'delay value must be positive')

    def test_retry_with_invalid_backoff(self):
        for value in (-1, 0, 0.1):
            with self.assertRaises(ValueError) as ctx:
                retry(backoff=value)(self.target)
            self.assertEqual(str(ctx.exception),
                             'backoff value must be a positive integer')
