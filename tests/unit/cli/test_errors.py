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
import sys
import tempfile
import xdg

from textwrap import dedent
from unittest import mock

import fixtures
from testtools.matchers import FileContains, MatchesRegex
from testscenarios import multiply_scenarios

import snapcraft.internal.errors
from snapcraft.internal.build_providers.errors import ProviderExecError
from snapcraft.cli._errors import exception_handler
from tests import unit


class TestSnapcraftError(snapcraft.internal.errors.SnapcraftError):

    fmt = "{message}"

    def __init__(self, message):
        super().__init__(message=message)

    def get_exit_code(self):
        return 123


class ErrorsBaseTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

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
        self.error_mock.assert_not_called()
        self.exit_mock.assert_called_once_with(1)
        self.print_exception_mock.assert_called_once_with(
            RuntimeError, mock.ANY, mock.ANY, file=_Tracefile(self)
        )

    def assert_exception_traceback_exit_1_without_raven(self):
        self.error_mock.assert_not_called()
        self.exit_mock.assert_called_once_with(1)
        self.print_exception_mock.assert_called_once_with(
            RuntimeError, mock.ANY, mock.ANY, file=sys.stdout
        )

    def assert_no_exception_traceback_exit_1_without_debug(self):
        self.error_mock.assert_not_called()
        self.exit_mock.assert_called_once_with(1)
        self.print_exception_mock.assert_not_called()


class ErrorsTestCase(ErrorsBaseTestCase):
    def setUp(self):
        super().setUp()

    @mock.patch.object(snapcraft.cli._errors, "RavenClient")
    def test_handler_no_raven_traceback_non_snapcraft_exceptions_debug(
        self, raven_client_mock
    ):
        snapcraft.cli._errors.RavenClient = None
        try:
            self.call_handler(RuntimeError("not a SnapcraftError"), True)
        except Exception:
            self.fail("Exception unexpectedly raised")

        self.assert_exception_traceback_exit_1_without_raven()

    def test_handler_catches_snapcraft_exceptions_no_debug(self):
        try:
            self.call_handler(TestSnapcraftError("is a SnapcraftError"), False)
        except Exception:
            self.fail("Exception unexpectedly raised")

        self.error_mock.assert_called_once_with("is a SnapcraftError")
        self.exit_mock.assert_called_once_with(123)
        self.print_exception_mock.assert_not_called()

    def test_handler_traces_snapcraft_exceptions_with_debug(self):
        try:
            self.call_handler(TestSnapcraftError("is a SnapcraftError"), True)
        except Exception:
            self.fail("Exception unexpectedly raised")

        self.error_mock.assert_not_called()
        self.exit_mock.assert_called_once_with(123)
        self.print_exception_mock.assert_called_once_with(
            TestSnapcraftError, mock.ANY, mock.ANY
        )


class ProviderErrorTest(ErrorsBaseTestCase):
    def setUp(self):
        super().setUp()

        patcher = mock.patch("shutil.move")
        self.move_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("traceback.print_exception")
        self.traceback_mock = patcher.start()
        self.addCleanup(patcher.stop)

    @mock.patch("os.path.isfile", return_value=False)
    def test_provider_error_legit(self, isfile_function):
        # Provider exception was raised in host environment, no crash file
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT", "host")
        )
        self._raise_exec_error()
        self.move_mock.assert_not_called()
        self.traceback_mock.assert_not_called()

    @mock.patch("os.path.isfile", return_value=True)
    def test_provider_error_outer(self, isfile_function):
        # Provider exception was raised in host environment with crash file
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT", "host")
        )
        self._raise_exec_error()
        self.assertThat(self.move_mock.call_count, Equals(1))
        self.traceback_mock.assert_not_called()

    @mock.patch("os.path.isfile", return_value=False)
    def test_provider_error_host(self, isfile_function):
        # Some other error raised in the host environment
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT", "host")
        )
        self._raise_other_error()
        self.move_mock.assert_not_called()
        self.assertThat(self.move_mock.call_count, Equals(1))

    @mock.patch("os.path.isfile", return_value=False)
    def test_provider_error_inner(self, isfile_function):
        # Error raised inside the build provider
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT", "managed-host")
        )
        self._raise_other_error()
        self.move_mock.assert_not_called()
        self.assertThat(self.traceback_mock.call_count, Equals(2))

    def _raise_exec_error(self):
        self.call_handler(
            ProviderExecError(provider_name="foo", command="bar", exit_code=2), False
        )

    def _raise_other_error(self):
        self.call_handler(KeyError, True)


class SendToSentryBaseTest(ErrorsBaseTestCase):
    def setUp(self):
        super().setUp()

        try:
            import raven  # noqa: F401
        except ImportError:
            self.skipTest("raven needs to be installed for this test.")

        patcher = mock.patch("click.prompt")
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

    yes_scenarios = [
        (answer, dict(answer=answer)) for answer in ["y", "Y", "YES", "yes", "Yes"]
    ]

    tty_scenarios = [("tty yes", dict(tty=True)), ("tty no", dict(tty=False))]

    scenarios = multiply_scenarios(yes_scenarios, tty_scenarios)

    def test_send(self):
        self.prompt_mock.return_value = self.answer
        self.mock_isatty.return_value = self.tty

        try:
            self.call_handler(RuntimeError("not a SnapcraftError"), True)
        except Exception:
            self.fail("Exception unexpectedly raised")

        self.raven_client_mock.assert_called_once_with(
            mock.ANY,
            transport=self.raven_request_mock,
            name="snapcraft",
            processors=mock.ANY,
            release=mock.ANY,
            auto_log_stacks=False,
        )

        # It we have a tty, then the trace should be saved to a file and sent to sentry.
        # If we don't have a tty, then the same should happen, but the trace should
        # also be printed.
        self.error_mock.assert_not_called()
        self.exit_mock.assert_called_once_with(1)

        expected_calls = [
            mock.call(RuntimeError, mock.ANY, mock.ANY, file=_Tracefile(self))
        ]

        if not self.tty:
            expected_calls.append(
                mock.call(RuntimeError, mock.ANY, mock.ANY, file=sys.stdout)
            )

        self.print_exception_mock.assert_has_calls(expected_calls, any_order=True)


class SendToSentryIsNoTest(SendToSentryBaseTest):

    no_scenarios = [
        (answer, dict(answer=answer)) for answer in ["n", "N", "NO", "no", "No"]
    ]

    tty_scenarios = [("tty yes", dict(tty=True)), ("tty no", dict(tty=False))]

    scenarios = multiply_scenarios(no_scenarios, tty_scenarios)

    def test_no_send(self):
        self.prompt_mock.return_value = self.answer
        self.mock_isatty.return_value = self.tty

        try:
            self.call_handler(RuntimeError("not a SnapcraftError"), True)
        except Exception:
            self.fail("Exception unexpectedly raised")

        self.raven_client_mock.assert_not_called()

        # It we have a tty, then the trace should be saved to a file and sent to sentry.
        # If we don't have a tty, then the same should happen, but the trace should
        # also be printed.
        self.error_mock.assert_not_called()
        self.exit_mock.assert_called_once_with(1)

        expected_calls = [
            mock.call(RuntimeError, mock.ANY, mock.ANY, file=_Tracefile(self))
        ]

        if not self.tty:
            expected_calls.append(
                mock.call(RuntimeError, mock.ANY, mock.ANY, file=sys.stdout)
            )

        self.print_exception_mock.assert_has_calls(expected_calls, any_order=True)


class SendToSentryIsAlwaysTest(SendToSentryBaseTest):

    always_scenarios = [
        (answer, dict(answer=answer))
        for answer in ["a", "A", "ALWAYS", "always", "Always"]
    ]

    tty_scenarios = [("tty yes", dict(tty=True)), ("tty no", dict(tty=False))]

    scenarios = multiply_scenarios(always_scenarios, tty_scenarios)

    def test_send_and_set_to_always(self):
        self.prompt_mock.return_value = self.answer
        self.mock_isatty.return_value = self.tty

        try:
            self.call_handler(RuntimeError("not a SnapcraftError"), True)
        except Exception:
            self.fail("Exception unexpectedly raised")

        self.raven_client_mock.assert_called_once_with(
            mock.ANY,
            transport=self.raven_request_mock,
            name="snapcraft",
            processors=mock.ANY,
            release=mock.ANY,
            auto_log_stacks=False,
        )
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
        # If we don't have a tty, then the same should happen, but the trace should
        # also be printed.
        self.error_mock.assert_not_called()
        self.exit_mock.assert_called_once_with(1)

        expected_calls = [
            mock.call(RuntimeError, mock.ANY, mock.ANY, file=_Tracefile(self))
        ]

        if not self.tty:
            expected_calls.append(
                mock.call(RuntimeError, mock.ANY, mock.ANY, file=sys.stdout)
            )

        self.print_exception_mock.assert_has_calls(expected_calls, any_order=True)


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
        self.raven_client_mock.assert_called_once_with(
            mock.ANY,
            transport=self.raven_request_mock,
            name="snapcraft",
            processors=mock.ANY,
            release=mock.ANY,
            auto_log_stacks=False,
        )
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
        self.raven_client_mock.assert_called_once_with(
            mock.ANY,
            transport=self.raven_request_mock,
            name="snapcraft",
            processors=mock.ANY,
            release=mock.ANY,
            auto_log_stacks=False,
        )

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
        try:
            self._test.assertThat(
                other.name, MatchesRegex("{}.*/trace.txt".format(tempfile.gettempdir()))
            )
        except AttributeError:
            return False
        return True
