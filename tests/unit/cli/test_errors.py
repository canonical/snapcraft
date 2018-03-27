# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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
from unittest import mock

import snapcraft.internal.errors
from snapcraft.cli._errors import exception_handler
from tests import unit


class TestSnapcraftError(snapcraft.internal.errors.SnapcraftError):

    fmt = '{message}'

    def __init__(self, message):
        super().__init__(message=message)

    def get_exit_code(self):
        return 123


class ErrorsTestCase(unit.TestCase):

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

    def assert_exception_traceback_exit_1_with_debug(self):
        self.error_mock.assert_not_called
        self.exit_mock.assert_called_once_with(1)
        self.print_exception_mock.assert_called_once_with(
            RuntimeError, mock.ANY, mock.ANY)

    def assert_no_exception_traceback_exit_1_without_debug(self):
        self.error_mock.assert_not_called
        self.exit_mock.assert_called_once_with(1)
        self.print_exception_mock.assert_not_called()

    @mock.patch('click.confirm', return_value=False)
    def test_handler_traceback_non_snapcraft_exceptions_no_debug(
            self, click_confirm_mock):
        """
        Verify that the traceback is printed given that raven is available.
        """
        try:
            self.call_handler(RuntimeError('not a SnapcraftError'), False)
        except Exception:
            self.fail('Exception unexpectedly raised')

        self.assert_exception_traceback_exit_1_with_debug()

    @mock.patch('click.confirm', return_value=False)
    def test_handler_traceback_non_snapcraft_exceptions_debug(
            self, click_confirm_mock):
        try:
            self.call_handler(RuntimeError('not a SnapcraftError'), True)
        except Exception:
            self.fail('Exception unexpectedly raised')

        self.assert_exception_traceback_exit_1_with_debug()

    @mock.patch.object(snapcraft.cli._errors, 'RavenClient')
    def test_handler_no_raven_traceback_non_snapcraft_exceptions_debug(
            self, raven_client_mock):
        snapcraft.cli._errors.RavenClient = None
        try:
            self.call_handler(RuntimeError('not a SnapcraftError'), True)
        except Exception:
            self.fail('Exception unexpectedly raised')

        self.assert_exception_traceback_exit_1_with_debug()

    @mock.patch('snapcraft.cli._errors.RavenClient')
    @mock.patch('snapcraft.cli._errors.RequestsHTTPTransport')
    @mock.patch('click.confirm', return_value=True)
    def test_handler_traceback_send_traceback_to_sentry(
            self, click_confirm_mock, raven_request_mock, raven_client_mock):
        try:
            self.call_handler(RuntimeError('not a SnapcraftError'), True)
        except Exception:
            self.fail('Exception unexpectedly raised')

        self.assert_exception_traceback_exit_1_with_debug()
        raven_client_mock.assert_called_once_with(
            mock.ANY, transport=raven_request_mock, processors=mock.ANY,
            auto_log_stacks=False)

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
