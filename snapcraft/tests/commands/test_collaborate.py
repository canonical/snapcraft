
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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

import logging

from unittest import mock

import fixtures

from snapcraft import (
    storeapi,
    tests
)

from snapcraft._store import collaborate


class CollaborateTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.fake_store = tests.fixture_setup.FakeStore()
        self.useFixture(self.fake_store)
        self.client = storeapi.StoreClient()
        patcher = mock.patch('snapcraft._store.Popen')
        self.popen_mock = patcher.start()
        process_mock = mock.Mock()
        process_mock.returncode = 0
        process_mock.communicate.return_value = [b'foo', b'']
        self.popen_mock.return_value = process_mock
        self.addCleanup(patcher.stop)

    def test_collaborate_success(self):
        self.client.login('dummy', 'test correct password')
        collaborate('ubuntu-core', 'keyname')

        self.popen_mock.assert_called_with(['snap', 'sign', '-k', 'keyname'],
                                           stderr=-1, stdin=-1, stdout=-1)
        self.assertIn('Signing developers assertion for ubuntu-core',
                      self.fake_logger.output)
        self.assertNotIn('Error signing developers assertion',
                         self.fake_logger.output)
        self.assertNotIn('Invalid response from the server',
                         self.fake_logger.output)

    def test_collaborate_snap_not_found(self):
        self.client.login('dummy', 'test correct password')

        err = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            collaborate,
            'notfound', 'key')

        self.assertIn("Snap 'notfound' was not found", str(err))

    def test_collaborate_snap_developer_not_found(self):
        self.client.login('dummy', 'test correct password')

        collaborate('core-no-dev', 'keyname')

        self.assertIn('Signing developers assertion for core-no-dev',
                      self.fake_logger.output)
        self.assertNotIn('Error signing developers assertion',
                         self.fake_logger.output)
        self.assertNotIn('Invalid response from the server',
                         self.fake_logger.output)

    def test_collaborate_bad_request(self):
        self.client.login('dummy', 'test correct password')
        err = self.assertRaises(
            storeapi.errors.StoreValidationError,
            collaborate,
            'badrequest', 'keyname')

        self.assertEqual(
                'Received error 400: "The given `snap-id` does not match '
                'the assertion\'s."', str(err))

    def test_collaborate_yes_revoke_uploads_request(self):
        patcher = mock.patch('builtins.input')
        mock_input = patcher.start()
        mock_input.return_value = 'y'
        self.client.login('dummy', 'test correct password')
        err = self.assertRaises(
            storeapi.errors.StoreValidationError,
            collaborate,
            'revoked', 'keyname')

        self.assertEqual(
            'Received error 409: "The assertion\'s `developers` would revoke '
            'existing uploads."', str(err))

