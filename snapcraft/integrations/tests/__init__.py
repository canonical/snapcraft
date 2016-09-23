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

import subprocess
from unittest import mock

from snapcraft import tests
from snapcraft.integrations import (
    requires_command_success,
    requires_path_exists,
)


class IntegrationsTestCase(tests.TestCase):

    @mock.patch('subprocess.check_call')
    def test_requires_command_success_not_found(self, mock_check_call):
        mock_check_call.side_effect = [FileNotFoundError()]

        with self.assertRaises(EnvironmentError) as raised:
            requires_command_success('foo').__enter__()

        self.assertEqual('`foo` not found.', str(raised.exception))

    @mock.patch('subprocess.check_call')
    def test_requires_command_success_error(self, mock_check_call):
        mock_check_call.side_effect = [
            subprocess.CalledProcessError(1, 'x')]

        with self.assertRaises(EnvironmentError) as raised:
            requires_command_success('foo').__enter__()

        self.assertEqual('`foo` failed.', str(raised.exception))

    def test_requires_command_success_broken(self):
        with self.assertRaises(TypeError) as raised:
            requires_command_success(1).__enter__()

        self.assertEqual('command must be a string.', str(raised.exception))

    def test_requires_path_exists_fails(self):
        with self.assertRaises(EnvironmentError) as raised:
            requires_path_exists('foo').__enter__()

        self.assertEqual(
            'Required path does not exist: foo', str(raised.exception))
