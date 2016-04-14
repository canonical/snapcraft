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
import json
from unittest import TestCase
from unittest.mock import patch

from snapcraft.storeapi.channels import get_channels, update_channels


class ChannelsAPITestCase(TestCase):

    def setUp(self):
        super(ChannelsAPITestCase, self).setUp()

        # setup patches
        oauth_session = 'snapcraft.storeapi.common.get_oauth_session'
        patcher = patch(oauth_session)
        self.mock_get_oauth_session = patcher.start()
        self.mock_session = self.mock_get_oauth_session.return_value
        self.addCleanup(patcher.stop)

        self.mock_get = self.mock_session.get
        self.mock_post = self.mock_session.post

        self.channels_data = [
            {'channel': 'stable', 'current': {'revision': 2, 'version': '1'}},
            {'channel': 'beta', 'current': {'revision': 4, 'version': '1.5'}},
            {'channel': 'edge', 'current': None},
        ]

    def set_channels_get_success_response(self):
        mock_response = self.mock_get.return_value
        mock_response.ok = True
        mock_response.json.return_value = self.channels_data

    def set_channels_get_error_response(self, error_msg):
        mock_response = self.mock_get.return_value
        mock_response.ok = False
        mock_response.text = error_msg

    def set_channels_post_success_response(self):
        mock_response = self.mock_post.return_value
        mock_response.ok = True
        mock_response.json.return_value = {
            'success': True, 'errors': [], 'channels': self.channels_data
        }

    def set_channels_post_failed_response(self, error_msg):
        mock_response = self.mock_post.return_value
        mock_response.ok = True
        mock_response.json.return_value = {
            'success': True, 'errors': [error_msg],
            'channels': self.channels_data
        }

    def set_channels_post_error_response(self, error_msg):
        mock_response = self.mock_post.return_value
        mock_response.ok = False
        mock_response.text = error_msg

    def test_get_channels(self):
        self.set_channels_get_success_response()

        data = get_channels(self.mock_session, 'package.name')

        expected = {
            'success': True,
            'errors': [],
            'data': self.channels_data,
        }
        self.assertEqual(data, expected)

    def test_get_channels_with_error_response(self):
        error_msg = 'some error'
        self.set_channels_get_error_response(error_msg)

        data = get_channels(self.mock_session, 'package.name')

        expected = {
            'success': False,
            'errors': [error_msg],
            'data': None,
        }
        self.assertEqual(data, expected)

    def test_get_channels_uses_environment_variables(self):
        with patch('snapcraft.storeapi.common.os.environ',
                   {'UBUNTU_STORE_API_ROOT_URL': 'http://example.com'}):
            get_channels(self.mock_session, 'package.name')
        self.mock_get.assert_called_once_with(
            'http://example.com/package-channels/package.name/', headers={})

    def test_update_channels(self):
        self.set_channels_post_success_response()

        data = update_channels(
            self.mock_session, 'package.name', {'stable': 2})

        expected = {
            'success': True,
            'errors': [],
            'data': self.channels_data,
        }
        self.assertEqual(data, expected)

    def test_update_channels_with_error_response(self):
        error_msg = 'some error'
        self.set_channels_post_error_response(error_msg)

        data = update_channels(
            self.mock_session, 'package.name', {'stable': 2})

        expected = {
            'success': False,
            'errors': [error_msg],
            'data': None,
        }
        self.assertEqual(data, expected)

    def test_update_channels_with_failed_response(self):
        error_msg = 'some error'
        self.set_channels_post_failed_response(error_msg)

        data = update_channels(
            self.mock_session, 'package.name', {'stable': 2})

        expected = {
            'success': True,
            'errors': [error_msg],
            'data': self.channels_data,
        }
        self.assertEqual(data, expected)

    def test_update_channels_uses_environment_variables(self):
        with patch('snapcraft.storeapi.common.os.environ',
                   {'UBUNTU_STORE_API_ROOT_URL': 'http://example.com'}):
            update_channels(
                self.mock_session, 'package.name', {'stable': 2})
        self.mock_post.assert_called_once_with(
            'http://example.com/package-channels/package.name/',
            data=json.dumps({'stable': 2}),
            headers={'Content-Type': 'application/json'})
