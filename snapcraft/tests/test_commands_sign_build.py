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
import glob
import json
import logging
from textwrap import dedent
from unittest import mock

import fixtures
from simplejson.scanner import JSONDecodeError

from snapcraft.main import main
from snapcraft import (
    storeapi,
    tests,
)


class SignBuildTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()
        self.fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(self.fake_logger)
        self.fake_terminal = tests.fixture_setup.FakeTerminal()
        self.useFixture(self.fake_terminal)

    @mock.patch.object(storeapi.SCAClient, 'get_account_information')
    @mock.patch.object(storeapi.StoreClient, 'sign_build')
    @mock.patch.object(storeapi.StoreClient, 'login')
    @mock.patch('subprocess.check_output')
    @mock.patch('getpass.getpass')
    @mock.patch('builtins.input')
    @mock.patch('snapcraft.internal.repo.is_package_installed')
    def test_sign_build_successfully(self, mock_installed, mock_input,
                                     mock_getpass, mock_check_output,
                                     mock_login,
                                     mock_get_account_information,
                                     mock_sign_build):
        mock_installed.return_value = True
        mock_check_output.side_effect = "lalalala"
        mock_get_account_information.return_value = {'account_id': 'abcd'}

        main(['init'])
        main([])
        #self.assertEqual("XXXXX", self.fake_logger.output)

        # create a sample snap and then sign its build
        main(['sign-build', 'my-snap-name_0.1_amd64.snap'])

        self.assertEqual("XXXXX", self.fake_logger.output)
