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

import os
import logging
from unittest import mock

import fixtures

from snapcraft.main import main
from snapcraft import (
    storeapi,
    tests,
)


class SignBuildTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.DEBUG)
        self.useFixture(self.fake_logger)
        self.fake_terminal = tests.fixture_setup.FakeTerminal()
        self.useFixture(self.fake_terminal)

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'login')
    @mock.patch('builtins.input')
    @mock.patch('getpass.getpass')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_sign_build_saved(self, mock_installed,
                              mock_input, mock_getpass, mock_login,
                              mock_get_account_information):
        account_info = {'account_id': 'abcd',
                        'snaps': {'16': {
                            'basic': {
                                'snap-id': 'snap-id',
                                }
                            }
                        }}

        mock_login.side_effect = [
            storeapi.errors.StoreTwoFactorAuthenticationRequired(), None]
        mock_input.side_effect = ['sample.person@canonical.com', '123456']
        mock_installed.return_value = True
        mock_get_account_information.return_value = account_info

        snap_path = os.path.join(
            os.path.dirname(tests.__file__), 'data', 'test-snap.snap')
        snap_build_path = snap_path + '-build'
        self.addCleanup(os.remove, snap_build_path)

        with self.assertRaises(SystemExit):
            main(['login'])
            # XXX: apparently failing to type registered key password at the moment
            main(['sign-build', snap_path, '--local', '-d'])
            msg = 'Assertion test_snap.snap-build saved to disk.'
            self.assertIn(msg, self.fake_logger.output)
