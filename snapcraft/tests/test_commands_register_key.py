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
import os.path
import shutil
import subprocess
from unittest import mock

import fixtures
from simplejson.scanner import JSONDecodeError

from snapcraft.main import main
from snapcraft import (
    storeapi,
    tests,
)


class TemporaryGPGFixture(fixtures.Fixture):
    """Set up a temporary local GPG environment with sample data."""

    def _setUp(self):
        home = self.useFixture(fixtures.TempDir()).path
        gpg_conf_path = os.path.join(home, 'gpg.conf')
        with open(gpg_conf_path, 'w') as gpg_conf:
            gpg_conf.write(
                'no-use-agent\n'
                'no-auto-check-trustdb\n')
        for name in 'pubring.gpg', 'secring.gpg':
            shutil.copy2(
                os.path.join(
                    os.path.dirname(tests.__file__), 'data', 'sample-' + name),
                os.path.join(home, name))
        self.useFixture(fixtures.EnvironmentVariable('GNUPGHOME', home))


class RegisterKeyTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.useFixture(TemporaryGPGFixture())

    def test_register_key_without_login_raises_exception(self):
        with self.assertRaises(SystemExit) as raised:
            main(['register-key', 'sample.person@canonical.com'])

        self.assertEqual(1, raised.exception.code)
        self.assertIn(
            'No valid credentials found. Have you run "snapcraft login"?\n',
            self.fake_logger.output)

    def test_register_key_successfully(self):
        with mock.patch.object(
                storeapi.SCAClient, 'register_key') as mock_register_key:
            main(['register-key', 'sample.person@canonical.com'])

        self.assertEqual(
            'Registering GPG key ...\n'
            'Done. The GPG key 6191 1EE6 0198 CD22 8462  '
            '19F6 A337 CC09 F1F1 2F4C will be expected for signing your '
            'assertions.\n',
            self.fake_logger.output)

        self.assertEqual(1, mock_register_key.call_count)
        gpg_output = subprocess.check_output(
            ['gpg'], input=mock_register_key.call_args[0][0]).decode('ascii')
        self.assertIn(
            '2016-07-29 Sample Person <sample.person@canonical.com>',
            gpg_output)

    def test_register_key_failed(self):
        response = mock.Mock()
        response.json.side_effect = JSONDecodeError('mock-fail', 'doc', 1)
        response.status_code = 500
        response.reason = 'Internal Server Error'
        with mock.patch.object(
                storeapi.SCAClient, 'register_key') as mock_register_key:
            mock_register_key.side_effect = (
                storeapi.errors.StoreKeyRegistrationError(response))
            with self.assertRaises(SystemExit) as raised:
                main(['register-key', 'sample.person@canonical.com'])

        self.assertEqual(1, raised.exception.code)
        self.assertIn('500 Internal Server Error\n', self.fake_logger.output)

    def test_register_key_excludes_dsa(self):
        with self.assertRaises(SystemExit) as raised:
            main(['register-key', 'dsa.person@canonical.com'])

        self.assertEqual(1, raised.exception.code)
        self.assertIn(
            'You have no usable GPG secret keys matching '
            '"dsa.person@canonical.com".\n',
            self.fake_logger.output)

    def test_register_key_excludes_weak_rsa(self):
        with self.assertRaises(SystemExit) as raised:
            main(['register-key', 'rsa-2048.person@canonical.com'])

        self.assertEqual(1, raised.exception.code)
        self.assertIn(
            'You have no usable GPG secret keys matching '
            '"rsa-2048.person@canonical.com".\n',
            self.fake_logger.output)

    def test_register_key_select_key(self):
        patcher = mock.patch('builtins.input')
        mock_input = patcher.start()
        mock_input.return_value = '2'
        self.addCleanup(patcher.stop)

        patcher = mock.patch('builtins.print')
        mock_print = patcher.start()
        self.addCleanup(patcher.stop)

        with mock.patch.object(
                storeapi.SCAClient, 'register_key') as mock_register_key:
            main(['register-key'])

        mock_print.assert_has_calls([
            mock.call('Select a key:'),
            mock.call(),
            mock.call('1. 4096R: '
                      '6191 1EE6 0198 CD22 8462  19F6 A337 CC09 F1F1 2F4C'),
            mock.call('   Sample Person <sample.person@canonical.com>'),
            mock.call('2. 4096R: '
                      '4EA2 B48B FB44 8064 7BCC  8B54 7929 3F36 2153 83B8'),
            mock.call('   Another Person <another.person@canonical.com>'),
            mock.call(),
            ])
        mock_input.assert_called_once_with('Key number: ')

        self.assertEqual(
            'Registering GPG key ...\n'
            'Done. The GPG key 4EA2 B48B FB44 8064 7BCC  '
            '8B54 7929 3F36 2153 83B8 will be expected for signing your '
            'assertions.\n',
            self.fake_logger.output)

        self.assertEqual(1, mock_register_key.call_count)
        gpg_output = subprocess.check_output(
            ['gpg'], input=mock_register_key.call_args[0][0]).decode('ascii')
        self.assertIn(
            '2016-08-12 Another Person <another.person@canonical.com>',
            gpg_output)
