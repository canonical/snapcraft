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
    tests,
)


class ValidateTestCase(tests.TestCase):

    command_name = 'validate'

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

    def test_validate_success(self):
        self.client.login('dummy', 'test correct password')

        main([self.command_name, 'ubuntu-core', "ubuntu-core=3",
              "test-snap=4"])

        self.assertIn('Signing validation ubuntu-core=3',
                      self.fake_terminal.getvalue())
        self.assertIn('Signing validation test-snap=4',
                      self.fake_terminal.getvalue())

    def test_validate_with_key(self):
        self.client.login('dummy', 'test correct password')

        main([self.command_name, 'ubuntu-core', "ubuntu-core=3",
              "test-snap=4", "--key-name=keyname"])
        self.popen_mock.assert_called_with(['snap', 'sign', '-k', 'keyname'],
                                           stderr=-1, stdin=-1, stdout=-1)

        self.assertIn('Signing validation ubuntu-core=3',
                      self.fake_terminal.getvalue())
        self.assertIn('Signing validation test-snap=4',
                      self.fake_terminal.getvalue())

    def test_validate_from_branded_store(self):
        # Validating snaps from a branded store requires setting
        # `SNAPCRAFT_UBUNTU_STORE` environment variable to the store 'slug'.
        self.client.login('dummy', 'test correct password')
        self.useFixture(
            fixtures.EnvironmentVariable(
                'SNAPCRAFT_UBUNTU_STORE', 'Test-Branded'))

        main([self.command_name, 'ubuntu-core', 'test-snap-branded-store=1'])

        self.assertIn('Signing validation test-snap-branded-store=1',
                      self.fake_terminal.getvalue())

    def test_validate_unknown_snap(self):
        self.client.login('dummy', 'test correct password')

        self.assertRaises(
            SystemExit,
            main,
            [self.command_name, 'notfound', "ubuntu-core=3", "test-snap=4"])

        self.assertIn("Snap 'notfound' was not found", self.fake_logger.output)

    def test_validate_bad_argument(self):
        self.client.login('dummy', 'test correct password')

        self.assertRaises(
            SystemExit,
            main,
            [self.command_name, 'ubuntu-core', "ubuntu-core=foo"])

        self.assertIn('format must be name=revision',
                      self.fake_logger.output)

    def test_no_login(self):
        self.assertRaises(
            SystemExit,
            main,
            [self.command_name, 'ubuntu-core', "ubuntu-core=3", "test-snap=4"])
        self.assertIn('No valid credentials found. Have you run',
                      self.fake_logger.output)
