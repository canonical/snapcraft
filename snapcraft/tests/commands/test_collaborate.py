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
from testtools.matchers import Equals

from snapcraft import (
    storeapi,
    tests
)
from snapcraft import _store


class CollaborateBaseTestCase(tests.TestCase):

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
        patcher = mock.patch('snapcraft._store._edit_collaborators')
        self.edit_collaborators_mock = patcher.start()
        self.addCleanup(patcher.stop)


class CollaborateTestCase(CollaborateBaseTestCase):

    def test_collaborate_success(self):
        self.edit_collaborators_mock.return_value = {
            'developers': [{
                'developer-id': 'dummy-id',
                'since': '2015-07-19 19:30:00'}]
        }
        self.client.login('dummy', 'test correct password')
        _store.collaborate('ubuntu-core', 'keyname')

        self.popen_mock.assert_called_with(['snap', 'sign', '-k', 'keyname'],
                                           stderr=-1, stdin=-1, stdout=-1)
        self.assertNotIn('Error signing developers assertion',
                         self.fake_logger.output)
        self.assertNotIn('Invalid response from the server',
                         self.fake_logger.output)


class CollaborateErrorsTestCase(CollaborateBaseTestCase):

    def setUp(self):
        super().setUp()
        self.edit_collaborators_mock.return_value = {
            'developers': [{
                'developer-id': 'dummy-id',
                'since': '2015-07-19 19:30:00'}]
        }

    def test_collaborate_snap_not_found(self):
        self.client.login('dummy', 'test correct password')

        err = self.assertRaises(
            storeapi.errors.SnapNotFoundError,
            _store.collaborate,
            'notfound', 'key')

        self.assertIn("Snap 'notfound' was not found", str(err))

    def test_collaborate_snap_developer_not_found(self):
        self.client.login('dummy', 'test correct password')

        _store.collaborate('core-no-dev', 'keyname')

        self.assertNotIn('Error signing developers assertion',
                         self.fake_logger.output)
        self.assertNotIn('Invalid response from the server',
                         self.fake_logger.output)

    def test_collaborate_bad_request(self):
        self.client.login('dummy', 'test correct password')
        err = self.assertRaises(
            storeapi.errors.StoreValidationError,
            _store.collaborate,
            'badrequest', 'keyname')

        self.assertThat(
            str(err),
            Equals('Received error 400: "The given `snap-id` does not match '
                   'the assertion\'s."'))

    def test_collaborate_unchanged_collaborators(self):
        self.edit_collaborators_mock.return_value = [{
            'developer-id': 'test-dev-id',
            'since': '2017-02-10T08:35:00.000000Z',
            'until': '2018-02-10T08:35:00.000000Z'
        }]
        self.client.login('dummy', 'test correct password')
        _store.collaborate('test-snap-with-dev', 'keyname')

        self.assertIn('Aborting due to unchanged collaborators list.',
                      self.fake_logger.output)
