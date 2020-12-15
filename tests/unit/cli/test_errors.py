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

import snapcraft.cli.echo
import snapcraft.internal.errors
from snapcraft.cli._errors import (
    _get_exception_exit_code,
    _is_reportable_error,
    _print_exception_message,
    exception_handler,
)
from snapcraft.internal.build_providers.errors import ProviderExecError
from tests import fixture_setup, unit


class SnapcraftTError(snapcraft.internal.errors.SnapcraftError):

    fmt = "{message}"

    def __init__(self, message):
        super().__init__(message=message)

    def get_exit_code(self):
        return 123


class SnapcraftTException(snapcraft.internal.errors.SnapcraftException):
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

        patcher = mock.patch("snapcraft.cli._errors.echo.error")
        self.error_mock = patcher.start()
        self.addCleanup(patcher.stop)

    def test_snapcraft_exception_format_all(self):
        exception = SnapcraftTException()
        exception._brief = "something's strange, in the neighborhood"
        exception._resolution = "who you gonna call? ghostbusters!!"
        exception._details = "i ain't afraid of no ghosts"
        exception._docs_url = "https://docs.snapcraft.io/the-snapcraft-format/8337"

        _print_exception_message(exception)
        self.error_mock.assert_called_once_with(
            """something's strange, in the neighborhood

Recommended resolution:
who you gonna call? ghostbusters!!

Detailed information:
i ain't afraid of no ghosts

For more information, check out:
https://docs.snapcraft.io/the-snapcraft-format/8337"""
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
        exception._docs_url = "https://docs.snapcraft.io/the-snapcraft-format/8337"

        _print_exception_message(exception)
        self.error_mock.assert_called_once_with(
            """something's strange, in the neighborhood

Recommended resolution:
who you gonna call? ghostbusters!!

For more information, check out:
https://docs.snapcraft.io/the-snapcraft-format/8337"""
        )

    def test_snapcraft_exception_reportable(self):
        exception = SnapcraftTException()
        exception._brief = "something's strange, in the neighborhood"
        exc_info = (snapcraft.internal.errors.SnapcraftException, exception, None)

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

        patcher = mock.patch("snapcraft.cli._errors.echo.error")
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

    @mock.patch.object(snapcraft.cli._errors, "RavenClient")
    @mock.patch("snapcraft.internal.common.is_snap", return_value=False)
    def test_handler_no_raven_traceback_non_snapcraft_exceptions_debug(
        self, is_snap_mock, raven_client_mock
    ):
        snapcraft.cli._errors.RavenClient = None
        try:
            self.call_handler(RuntimeError("not a SnapcraftError"), True)
        except Exception:
            self.fail("Exception unexpectedly raised")

        self.error_mock.assert_called_once_with("not a SnapcraftError")
        self.exit_mock.assert_called_once_with(1)
        self.assert_print_exception_called_both_stdout_and_tempfile(RuntimeError)

    def test_handler_catches_snapcraft_exceptions_no_debug(self):
        try:
            self.call_handler(SnapcraftTError("is a SnapcraftError"), False)
        except Exception:
            self.fail("Exception unexpectedly raised")

        self.error_mock.assert_called_once_with("is a SnapcraftError")
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
    def test_provider_error_legit(self, isfile_function):
        # Provider exception was raised in host environment, no crash file
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT", "host")
        )
        self._raise_exec_error()
        self.move_mock.assert_not_called()
        self.print_exception_mock.assert_not_called()

    @mock.patch("os.path.isfile", return_value=True)
    def test_provider_error_outer(self, isfile_function):
        # Provider exception was raised in host environment with crash file
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT", "host")
        )
        self._raise_exec_error()

        # Only moved if reportable
        self.move_mock.assert_not_called()
        self.print_exception_mock.assert_not_called()

    @mock.patch("os.path.isfile", return_value=False)
    def test_provider_error_host(self, isfile_function):
        # Some other error raised in the host environment
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT", "host")
        )
        self._raise_other_error()
        self.move_mock.assert_not_called()
        self.assertThat(self.print_exception_mock.call_count, Equals(1))

    @mock.patch("os.path.isfile", return_value=False)
    @mock.patch.object(snapcraft.cli._errors, "RavenClient")
    def test_provider_error_inner(self, isfile_function, raven_client_mock):
        # Error raised inside the build provider
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT", "managed-host")
        )
        snapcraft.cli._errors.RavenClient = "something"
        self._raise_other_error()
        self.move_mock.assert_not_called()
        self.assertThat(self.print_exception_mock.call_count, Equals(2))

    def _raise_exec_error(self):
        self.call_handler(
            ProviderExecError(provider_name="foo", command="bar", exit_code=2), False
        )

    def _raise_other_error(self):
        self.call_handler(KeyError, False)


class SendToSentryBaseTest(ErrorsBaseTestCase):
    def setUp(self):
        super().setUp()

        try:
            import raven  # noqa: F401
        except ImportError:
            self.skipTest("raven needs to be installed for this test.")

        patcher = mock.patch("snapcraft.cli.echo.prompt")
        self.prompt_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("snapcraft.cli._errors.RequestsHTTPTransport")
        self.raven_request_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("snapcraft.cli._errors.RavenClient")
        self.raven_client_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_ENABLE_ERROR_REPORTING", "yes")
        )


class SendToSentryFails(SendToSentryBaseTest):
    def test_send_fails(self):
        self.prompt_mock.return_value = "yes"
        self.mock_isatty.return_value = True

        self.raven_client_mock().captureException.side_effect = RuntimeError(
            "raven is broken"
        )
        try:
            self.call_handler(RuntimeError("not a SnapcraftError"), True)
        except Exception:
            self.fail("Exception unexpectedly raised")


class SendToSentryIsYesTest(SendToSentryBaseTest):
    def assert_run(self, tty):
        self.prompt_mock.return_value = "YES"
        self.mock_isatty.return_value = tty

        try:
            self.call_handler(RuntimeError("not a SnapcraftError"), False)
        except Exception:
            self.fail("Exception unexpectedly raised")

        if tty:
            self.raven_client_mock.assert_called()
        else:
            # Cannot prompt if not connected to TTY.
            self.raven_client_mock.assert_not_called()

        # It we have a tty, then the trace should be saved to a file and sent to sentry.
        # If we don't have a tty, then the same should happen, but the trace should
        # also be printed.
        self.error_mock.assert_called_once_with("not a SnapcraftError")
        self.exit_mock.assert_called_once_with(1)

    def test_send_tty(self):
        self.assert_run(True)
        self.assert_print_exception_called_only_tracefile(RuntimeError)

    def test_send_no_tty(self):
        self.assert_run(False)
        self.assert_print_exception_called_both_stdout_and_tempfile(RuntimeError)


class SendToSentryIsNoTest(SendToSentryBaseTest):
    def assert_run(self, tty):
        self.prompt_mock.return_value = "NO"
        self.mock_isatty.return_value = tty

        try:
            self.call_handler(RuntimeError("not a SnapcraftError"), False)
        except Exception:
            self.fail("Exception unexpectedly raised")

        self.raven_client_mock.assert_not_called()

        # It we have a tty, then the trace should be saved to a file and sent to sentry.
        # If we don't have a tty, then the same should happen, but the trace should
        # also be printed.
        self.error_mock.assert_called_once_with("not a SnapcraftError")
        self.exit_mock.assert_called_once_with(1)

    def test_no_send_tty(self):
        self.assert_run(True)
        self.assert_print_exception_called_only_tracefile(RuntimeError)

    def test_no_send_no_tty(self):
        self.assert_run(False)
        self.assert_print_exception_called_both_stdout_and_tempfile(RuntimeError)


class SendToSentryIsAlwaysTest(SendToSentryBaseTest):
    def assert_run(self, tty):
        self.prompt_mock.return_value = "ALWAYS"
        self.mock_isatty.return_value = tty

        try:
            self.call_handler(RuntimeError("not a SnapcraftError"), False)
        except Exception:
            self.fail("Exception unexpectedly raised")

        if tty:
            self.raven_client_mock.assert_called()

            config_path = os.path.join(
                xdg.BaseDirectory.save_config_path("snapcraft"), "cli.cfg"
            )
            self.assertThat(
                config_path,
                FileContains(
                    dedent(
                        """\
                [Sentry]
                always_send = true

                """
                    )
                ),
            )

            # It we have a tty, then the trace should be saved to a file and sent to sentry.
            self.assert_print_exception_called_only_tracefile(RuntimeError)
        else:
            # Cannot prompt if not connected to TTY.
            self.raven_client_mock.assert_not_called()

            # If we don't have a tty, the trace should be printed and saved to file.
            self.assert_print_exception_called_both_stdout_and_tempfile(RuntimeError)

        self.error_mock.assert_called_once_with("not a SnapcraftError")
        self.exit_mock.assert_called_once_with(1)

    def test_send_and_set_to_always_tty(self):
        self.assert_run(True)

    def test_send_and_set_to_always_no_tty(self):
        self.assert_run(False)


class SendToSentryAlreadyAlwaysTest(SendToSentryBaseTest):
    def test_send_as_always(self):
        config_path = os.path.join(
            xdg.BaseDirectory.save_config_path("snapcraft"), "cli.cfg"
        )
        with open(config_path, "w") as f:
            f.write(
                dedent(
                    """\
                [Sentry]
                always_send = true

                """
                )
            )

        try:
            self.call_handler(RuntimeError("not a SnapcraftError"), True)
        except Exception:
            self.fail("Exception unexpectedly raised")

        self.assert_exception_traceback_exit_1_with_debug()
        self.raven_client_mock.assert_called()
        self.prompt_mock.assert_not_called()

    def test_send_with_config_error_does_not_save_always(self):
        self.prompt_mock.return_value = "ALWAYS"

        config_path = os.path.join(
            xdg.BaseDirectory.save_config_path("snapcraft"), "cli.cfg"
        )
        with open(config_path, "w") as f:
            f.write("bad data")

        try:
            self.call_handler(RuntimeError("not a SnapcraftError"), True)
        except Exception:
            self.fail("Exception unexpectedly raised")

        self.assert_exception_traceback_exit_1_with_debug()
        self.raven_client_mock.assert_called()

        # Given the corruption, ensure it hasn't been written to
        self.assertThat(config_path, FileContains("bad data"))


class SendToSentryDisabledTest(SendToSentryBaseTest):
    def test_disabled_no_send(self):
        self.prompt_mock.return_value = "yes"
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_ENABLE_ERROR_REPORTING", "no")
        )

        try:
            self.call_handler(RuntimeError("not a SnapcraftError"), True)
        except Exception:
            self.fail("Exception unexpectedly raised")

        self.assert_exception_traceback_exit_1_with_debug()
        self.raven_client_mock.assert_not_called()


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
