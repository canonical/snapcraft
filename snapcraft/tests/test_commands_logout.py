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
from snapcraft import tests


class LogoutCommandTestCase(tests.TestCase):

    @mock.patch('snapcraft.config.clear_config')
    def test_successful_login_saves_config(self, mock_clear):
        fake_logger = fixtures.FakeLogger(level=logging.INFO)
        self.useFixture(fake_logger)

        main(['logout'])

        self.assertEqual(
            'Clearing credentials for Ubuntu One SSO.\n'
            'Credentials cleared.\n',
            fake_logger.output)
        mock_clear.assert_called_once_with()
