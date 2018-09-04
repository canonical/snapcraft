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
import xdg

from textwrap import dedent
from unittest import mock

import fixtures
from testtools.matchers import FileContains

import snapcraft.internal.errors
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

        patcher = mock.patch("snapcraft.cli._errors.echo.error")
        self.error_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch("traceback.print_exception")
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
            RuntimeError, mock.ANY, mock.ANY, file=mock.ANY
        )

    def assert_exception_traceback_exit_1_without_raven(self):
        self.error_mock.assert_not_called
        self.exit_mock.assert_called_once_with(1)
        self.print_exception_mock.assert_called_once_with(
            RuntimeError, mock.ANY, mock.ANY
        )

    def assert_no_exception_traceback_exit_1_without_debug(self):
        self.error_mock.assert_not_called
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
        self.print_exception_mock.assert_not_called

    def test_handler_traces_snapcraft_exceptions_with_debug(self):
        try:
            self.call_handler(TestSnapcraftError("is a SnapcraftError"), True)
        except Exception:
            self.fail("Exception unexpectedly raised")

        self.error_mock.assert_not_called
        self.exit_mock.assert_not_called
        self.print_exception_mock.assert_called_once_with(
            TestSnapcraftError, mock.ANY, mock.ANY
        )


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


class SendToSentryIsYesTest(SendToSentryBaseTest):

    scenarios = [
        (answer, dict(answer=answer)) for answer in ["y", "Y", "YES", "yes", "Yes"]
    ]

    def test_send(self):
        self.prompt_mock.return_value = self.answer

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


class SendToSentryIsNoTest(SendToSentryBaseTest):

    scenarios = [
        (answer, dict(answer=answer)) for answer in ["n", "N", "NO", "no", "No"]
    ]

    def test_no_send(self):
        self.prompt_mock.return_value = self.answer

        try:
            self.call_handler(RuntimeError("not a SnapcraftError"), True)
        except Exception:
            self.fail("Exception unexpectedly raised")

        self.assert_exception_traceback_exit_1_with_debug()
        self.raven_client_mock.assert_not_called()


class SendToSentryIsAlwaysTest(SendToSentryBaseTest):

    scenarios = [
        (answer, dict(answer=answer))
        for answer in ["a", "A", "ALWAYS", "always", "Always"]
    ]

    def test_send_and_set_to_always(self):
        self.prompt_mock.return_value = self.answer

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
