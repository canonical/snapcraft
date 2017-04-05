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

import logging

from unittest import mock

import fixtures

from snapcraft.main import main
from snapcraft import (
    storeapi,
    tests
)


class CollaborateTestCase(tests.TestCase):

    command_name = 'collaborate'

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.fake_store = tests.fixture_setup.FakeStore()
        self.useFixture(self.fake_store)
        self.client = storeapi.StoreClient()
        self.fake_terminal = tests.fixture_setup.FakeTerminal()
        self.useFixture(self.fake_terminal)
        patcher = mock.patch('snapcraft._store.Popen')
        self.popen_mock = patcher.start()
        rv_mock = mock.Mock()
        rv_mock.returncode = 0
        rv_mock.communicate.return_value = [b'foo', b'']
        self.popen_mock.return_value = rv_mock
        self.addCleanup(patcher.stop)

    def test_collaborate_success(self):
        self.client.login('dummy', 'test correct password')
        main(['--debug', self.command_name, 'ubuntu-core'])
        self.assertIn('Signing developers assertion for good',
                      self.fake_terminal.getvalue())
        self.assertNotIn('Error signing developers assertion',
                         self.fake_terminal.getvalue())
        self.assertNotIn('Invalid response from the server',
                         self.fake_terminal.getvalue())

    def test_collaborate_success_with_key(self):
        self.client.login('dummy', 'test correct password')
        main(['--debug', self.command_name, 'ubuntu-core',
              '--key-name=keyname'])
        self.popen_mock.assert_called_with(['snap', 'sign', '-k', 'keyname'],
                                           stderr=-1, stdin=-1, stdout=-1)
        self.assertIn('Signing developers assertion for good',
                      self.fake_terminal.getvalue())
        self.assertNotIn('Error signing developers assertion',
                         self.fake_terminal.getvalue())
        self.assertNotIn('Invalid response from the server',
                         self.fake_terminal.getvalue())

    def test_collaborate_snap_not_found(self):
        self.client.login('dummy', 'test correct password')

        self.assertRaises(
            SystemExit,
            main,
            [self.command_name, 'notfound'])

        self.assertIn("Snap 'notfound' was not found", self.fake_logger.output)

    def test_collaborate_snap_developer_not_found(self):
        self.client.login('dummy', 'test correct password')

        main(['--debug', self.command_name, 'core-no-dev'])

        self.assertIn('Signing developers assertion for no-dev',
                      self.fake_terminal.getvalue())
        self.assertNotIn('Error signing developers assertion',
                         self.fake_terminal.getvalue())
        self.assertNotIn('Invalid response from the server',
                         self.fake_terminal.getvalue())

    def test_collaborate_bad_request(self):
        self.client.login('dummy', 'test correct password')
        err = self.assertRaises(
            storeapi.errors.StoreValidationError,
            main,
            ['--debug', self.command_name, 'badrequest'])

        self.assertEqual(
                'Received error 400: "The given `snap-id` does not match '
                'the assertion\'s."', str(err))
