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

import sys

from snapcraft.cli._errors import exception_handler
import snapcraft.internal.errors

import testtools
from unittest import mock

from snapcraft import tests


class TestSnapcraftError(snapcraft.internal.errors.SnapcraftError):

    fmt = '{message}'

    def __init__(self, message):
        super().__init__(message=message)

    def get_exit_code(self):
        return 123


class ErrorsTestCase(tests.TestCase):

    def setUp(self):
        super().setUp()

        patcher = mock.patch('sys.exit')
        self.exit_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('snapcraft.cli._errors.echo.error')
        self.error_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch('traceback.print_exception')
        self.print_exception_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def call_handler(self, exception, debug):
        try:
            raise exception
        except Exception:
            exception_handler(*sys.exc_info(), debug=debug)

    def test_handler_raises_non_snapcraft_exceptions(self):
        with testtools.ExpectedException(RuntimeError, 'not a SnapcraftError'):
            self.call_handler(RuntimeError('not a SnapcraftError'), False)

        with testtools.ExpectedException(RuntimeError, 'not a SnapcraftError'):
            self.call_handler(RuntimeError('not a SnapcraftError'), True)

        self.error_mock.assert_not_called
        self.exit_mock.assert_not_called
        self.print_exception_mock.assert_not_called

    def test_handler_catches_snapcraft_exceptions_no_debug(self):
        try:
            self.call_handler(TestSnapcraftError('is a SnapcraftError'), False)
        except Exception:
            self.fail('Exception unexpectedly raised')

        self.error_mock.assert_called_once_with('is a SnapcraftError')
        self.exit_mock.assert_called_once_with(123)
        self.print_exception_mock.assert_not_called

    def test_handler_traces_snapcraft_exceptions_with_debug(self):
        try:
            self.call_handler(TestSnapcraftError('is a SnapcraftError'), True)
        except Exception:
            self.fail('Exception unexpectedly raised')

        self.error_mock.assert_not_called
        self.exit_mock.assert_not_called
        self.print_exception_mock.assert_called_once_with(
            TestSnapcraftError, mock.ANY, mock.ANY)
