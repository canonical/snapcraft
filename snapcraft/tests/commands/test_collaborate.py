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
import yaml

from snapcraft import (
    storeapi,
    tests
)

from snapcraft import _store


class EditCollaboratorsTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        patcher = mock.patch('subprocess.check_call')
        patcher.start()
        self.addCleanup(patcher.stop)

    def test_edit_collaborators_must_write_header_and_developers(self):
        developers_from_assertion = [
            {'developer-id': 'test-dev-id1',
             'since': '2017-02-10T08:35:00.000000Z',
             'until': '2018-02-10T08:35:00.000000Z'},
            {'developer-id': 'test-dev-id2',
             'since': '2016-02-10T08:35:00.000000Z',
             'until': '2019-02-10T08:35:00.000000Z'},
        ]

        existing_developers = ("developers:\n"
                               "- developer-id: test-dev-id1\n"
                               "  since: '2017-02-10 08:35:00'\n"
                               "  until: '2018-02-10 08:35:00'\n"
                               "- developer-id: test-dev-id2\n"
                               "  since: '2016-02-10 08:35:00'\n"
                               "  until: '2019-02-10 08:35:00'\n")
        expected_written = (_store._COLLABORATION_HEADER + '\n' +
                            existing_developers)

        with mock.patch(
                'builtins.open',
                new_callable=mock.mock_open,
                read_data=expected_written) as mock_open:
            developers_for_assertion = _store._edit_collaborators(
                developers_from_assertion)

        written = ''
        for call in mock_open().write.call_args_list:
            written += str(call.call_list()[0][0][0])
        self.assertEqual(expected_written, written)
        self.assertEqual(developers_for_assertion, developers_from_assertion)

    def test_edit_collaborators_must_return_new_developers(self):
        developers_from_assertion = [
            {'developer-id': 'test-dev-id1',
             'since': '2017-02-10T08:35:00.000000Z',
             'until': '2018-02-10T08:35:00.000000Z'},
        ]

        new_developers = ("developers:\n"
                          "- developer-id: test-dev-id1\n"
                          "  since: '2017-02-10 08:35:00'\n"
                          "  until: '2018-02-10 08:35:00'\n"
                          "- developer-id: test-dev-id2\n"
                          "  since: '2016-02-10 08:35:00'\n"
                          "  until: '2019-02-10 08:35:00'\n")

        with mock.patch(
                'builtins.open',
                new_callable=mock.mock_open,
                read_data=new_developers) as mock_open:
            developers_for_assertion = _store._edit_collaborators(
                developers_from_assertion)

        expected_developers = developers_from_assertion + [{
            'developer-id': 'test-dev-id2',
            'since': '2016-02-10T08:35:00.000000Z',
            'until': '2019-02-10T08:35:00.000000Z',
        }]
        self.assertEqual(developers_for_assertion, expected_developers)


class EditCollaboratorsOpenEditorTestCase(tests.TestCase):

    scenarios = (
        ('default', {'editor': None, 'expected': 'vi'}),
        ('non-default', {'editor': 'test-editor', 'expected': 'test-editor'})
    )

    def setUp(self):
        super().setUp()
        patcher = mock.patch('subprocess.check_call')
        self.check_call_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_edit_collaborators_must_open_editor(self):
        self.useFixture(fixtures.EnvironmentVariable('EDITOR', self.editor))
        _store._edit_collaborators({})
        self.check_call_mock.assert_called_with([self.expected, mock.ANY])


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
        self.assertIn('Signing developers assertion for ubuntu-core',
                      self.fake_logger.output)
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
            _store.collaborate,
            'badrequest', 'keyname')

        self.assertEqual(
                'Received error 400: "The given `snap-id` does not match '
                'the assertion\'s."', str(err))
