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
import ssoclient.v2 as sso

from snapcraft.main import main
from snapcraft import (
    config,
    tests
)


class LoginCommandTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)

        patcher = mock.patch('builtins.input')
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('builtins.print')
        patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('getpass.getpass')
        patcher.start()
        self.addCleanup(patcher.stop)

#        patcher = mock.patch('snapcraft._store.save_config')
#        self.mock_save = patcher.start()
#        self.addCleanup(patcher.stop)

        patcher = mock.patch.object(sso.V2ApiClient, 'login')
        self.mock_login = patcher.start()
        self.addCleanup(patcher.stop)

    def test_successful_login_saves_config(self):
        response = {
            'success': True,
            'consumer_key': 'test_consumer_key',
            'consumer_secret': 'test_consumer_secret',
            'token_key': 'test_token_key',
            'token_secret': 'test_token_secret',
        }
        self.mock_login.return_value = response

        main(['login'])

        self.assertEqual(
            'Authenticating against Ubuntu One SSO.\n'
            'Login successful.\n',
            self.fake_logger.output)

        conf = config.Config()
        self.assertEqual('test_consumer_key', conf.get('consumer_key'))
        self.assertEqual('test_consumer_secret', conf.get('consumer_secret'))
        self.assertEqual('test_token_key', conf.get('token_key'))
        self.assertEqual('test_token_secret', conf.get('token_secret'))

    def test_failed_login_does_not_save_config(self):
        self.mock_login.side_effect = sso.ApiException(
            response=type('obj', (object,),  {'status_code': 401}))

        main(['login'])

        self.assertEqual(
            'Authenticating against Ubuntu One SSO.\n'
            'Login failed.\n',
            self.fake_logger.output)
        self.assertTrue(config.Config().is_empty())
