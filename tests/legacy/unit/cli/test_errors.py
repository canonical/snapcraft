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

import os
import re
import sys
import tempfile
from textwrap import dedent
from unittest import mock

import fixtures
import xdg
from testtools.matchers import Equals, FileContains

import snapcraft_legacy.cli.echo
import snapcraft_legacy.internal.errors
from snapcraft_legacy.cli._errors import (
    _get_exception_exit_code,
    _is_reportable_error,
    _print_exception_message,
    exception_handler,
)
from snapcraft_legacy.internal.build_providers.errors import ProviderExecError
from tests.legacy import fixture_setup, unit


class SnapcraftTError(snapcraft_legacy.internal.errors.SnapcraftError):
    fmt = "{message}"

    def __init__(self, message):
        super().__init__(message=message)

    def get_exit_code(self):
        return 123


class SnapcraftTException(snapcraft_legacy.internal.errors.SnapcraftException):
    def __init__(self):
        self._brief = ""
        self._resolution = ""
        self._details = super().get_details()
        self._docs_url = super().get_docs_url()
        self._reportable = super().get_reportable()
        self._exit_code = super().get_exit_code()

    def get_brief(self):
        return self._brief

    def get_resolution(self):
        return self._resolution

    def get_details(self):
        return self._details

    def get_docs_url(self):
        return self._docs_url

    def get_exit_code(self):
        return self._exit_code

    def get_reportable(self):
        return self._reportable


class TestSnapcraftExceptionHandling(unit.TestCase):
    def setUp(self):
        super().setUp()

        patcher = mock.patch("snapcraft_legacy.cli._errors.echo.error")
        self.error_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_snapcraft_exception_format_all(self):
        exception = SnapcraftTException()
        exception._brief = "something's strange, in the neighborhood"
        exception._resolution = "who you gonna call? ghostbusters!!"
        exception._details = "i ain't afraid of no ghosts"
        exception._docs_url = "https://documentation.ubuntu.com/snapcraft/stable/reference/project-file"

        _print_exception_message(exception)
        self.error_mock.assert_called_once_with(
            """something's strange, in the neighborhood

Recommended resolution:
who you gonna call? ghostbusters!!

Detailed information:
i ain't afraid of no ghosts

For more information, check out:
https://documentation.ubuntu.com/snapcraft/stable/reference/project-file"""
        )

    def test_snapcraft_exception_minimal(self):
        exception = SnapcraftTException()
        exception._brief = "something's strange, in the neighborhood"
        exception._resolution = ""
        _print_exception_message(exception)
        self.error_mock.assert_called_once_with(
            """something's strange, in the neighborhood"""
        )

    def test_snapcraft_exception_minimal_with_resolution(self):
        exception = SnapcraftTException()
        exception._brief = "something's strange, in the neighborhood"
        exception._resolution = "who you gonna call? ghostbusters!!"
        _print_exception_message(exception)
        self.error_mock.assert_called_once_with(
            """something's strange, in the neighborhood

Recommended resolution:
who you gonna call? ghostbusters!!"""
        )

    def test_snapcraft_exception_minimal_with_resolution_and_url(self):
        exception = SnapcraftTException()
        exception._brief = "something's strange, in the neighborhood"
        exception._resolution = "who you gonna call? ghostbusters!!"
        exception._docs_url = "https://documentation.ubuntu.com/snapcraft/stable/reference/project-file"

        _print_exception_message(exception)
        self.error_mock.assert_called_once_with(
            """something's strange, in the neighborhood

Recommended resolution:
who you gonna call? ghostbusters!!

For more information, check out:
https://documentation.ubuntu.com/snapcraft/stable/reference/project-file"""
        )

    def test_snapcraft_exception_reportable(self):
        exception = SnapcraftTException()
        exception._brief = "something's strange, in the neighborhood"
        exc_info = (
            snapcraft_legacy.internal.errors.SnapcraftException,
            exception,
            None,
        )

        # Test default (is false).
        self.assertFalse(_is_reportable_error(exc_info))

        # Test false.
        exception._reportable = False
        self.assertFalse(_is_reportable_error(exc_info))

        # Test true.
        exception._reportable = True
        self.assertTrue(_is_reportable_error(exc_info))

    def test_snapcraft_exception_exit_code(self):
        exception = SnapcraftTException()
        exception._brief = "something's strange, in the neighborhood"
        exception._resolution = "who you gonna call? ghostbusters!!"

        # Test default.
        self.assertEquals(2, _get_exception_exit_code(exception))

        # Test override.
        exception._exit_code = 50
        self.assertEquals(50, _get_exception_exit_code(exception))


class ReportableErrorTests(unit.TestCase):
    def test_keyboard_interrupt(self):
        exc_info = (KeyboardInterrupt, KeyboardInterrupt(), None)
        self.assertFalse(_is_reportable_error(exc_info))


class ErrorsBaseTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.useFixture(fixture_setup.FakeSnapcraftIsASnap())

        patcher = mock.patch("sys.exit")
        self.exit_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("builtins.print")
        self.print_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("snapcraft_legacy.cli._errors.echo.error")
        self.error_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("traceback.print_exception")
        self.print_exception_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("sys.stdout.isatty", return_value=True)
        self.mock_isatty = patcher.start()
        self.addCleanup(patcher.stop)

    def call_handler(self, exception, debug):
        try:
            raise exception
        except Exception:
            exception_handler(*sys.exc_info(), debug=debug)

    def assert_exception_traceback_exit_1_with_debug(self):
        self.assertThat(self.error_mock.call_count, Equals(1))
        self.exit_mock.assert_called_once_with(1)

        # called twice - once for stdout, once for trace file
        self.assertThat(self.print_exception_mock.call_count, Equals(2))

    def assert_no_exception_traceback_exit_1_without_debug(self):
        self.assertThat(self.error_mock.call_count, Equals(1))
        self.exit_mock.assert_called_once_with(1)
        self.print_exception_mock.assert_not_called()

    def assert_print_exception_called_only_tracefile(self, error):
        self.print_exception_mock.assert_called_once_with(
            error, mock.ANY, mock.ANY, file=_Tracefile(self)
        )

    def assert_print_exception_called_both_stdout_and_tempfile(self, error):
        expected_calls = [
            mock.call(error, mock.ANY, mock.ANY, file=_Tracefile(self)),
            # TODO setup pytest runner for this to work.
            # mock.call(error, mock.ANY, mock.ANY, file=_Stdout(self)),
        ]

        self.print_exception_mock.assert_has_calls(expected_calls, any_order=True)


class ErrorsTestCase(ErrorsBaseTestCase):
    def setUp(self):
        super().setUp()

    def test_handler_catches_snapcraft_exceptions_no_debug(self):
        try:
            self.call_handler(SnapcraftTError("is a SnapcraftError"), False)
        except Exception:
            self.fail("Exception unexpectedly raised")

        self.error_mock.assert_called_once()
        args, _ = self.error_mock.call_args
        self.assertIn("is a SnapcraftError", args[0])
        self.exit_mock.assert_called_once_with(123)
        self.print_exception_mock.assert_not_called()

    def test_handler_traces_snapcraft_exceptions_with_debug(self):
        try:
            self.call_handler(SnapcraftTError("is a SnapcraftError"), True)
        except Exception:
            self.fail("Exception unexpectedly raised")

        self.error_mock.assert_called_once_with("is a SnapcraftError")
        self.exit_mock.assert_called_once_with(123)
        self.assert_print_exception_called_both_stdout_and_tempfile(SnapcraftTError)


class ProviderErrorTest(ErrorsBaseTestCase):
    def setUp(self):
        super().setUp()

        patcher = mock.patch("shutil.move")
        self.move_mock = patcher.start()
        self.addCleanup(patcher.stop)


    @mock.patch("os.path.isfile", return_value=False)
    def test_provider_error_host(self, isfile_function):
        # Some other error raised in the host environment
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT", "host")
        )
        self._raise_other_error()
        self.move_mock.assert_not_called()
        self.assertThat(self.print_exception_mock.call_count, Equals(1))




    def _raise_other_error(self):
        self.call_handler(KeyError, False)


class SendToSentryBaseTest(ErrorsBaseTestCase):
    def setUp(self):
        super().setUp()

        try:
            import raven  # noqa: F401
        except ImportError:
            self.skipTest("raven needs to be installed for this test.")

        patcher = mock.patch("snapcraft_legacy.cli.echo.prompt")
        self.prompt_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("snapcraft_legacy.cli._errors.RequestsHTTPTransport")
        self.raven_request_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("snapcraft_legacy.cli._errors.RavenClient")
        self.raven_client_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_ENABLE_ERROR_REPORTING", "yes")
        )




class _Tracefile:
    def __init__(self, test):
        self._test = test

    def __eq__(self, other):
        if not hasattr(other, "name"):
            return False

        if re.match("{}.*/trace.txt".format(tempfile.gettempdir()), other.name):
            return True

        return False


class _Stdout:
    def __init__(self, test):
        self._test = test

    def __eq__(self, other):
        if not hasattr(other, "name"):
            return False

        if other.name == "<stdout>":
            return True

        return False
