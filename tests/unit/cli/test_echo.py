# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2019 Canonical Ltd
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

from textwrap import dedent
from unittest import mock

import fixtures

from snapcraft.cli import echo
from tests import unit


class EchoTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.click_confirm = self.useFixture(
            fixtures.MockPatch("click.confirm", return_value=True)
        )
        self.click_prompt = self.useFixture(
            fixtures.MockPatch("click.prompt", return_value=True)
        )

    @mock.patch("sys.stdin.isatty", return_value=False)
    def test_is_tty_disconnected(self, tty_mock):
        result = echo.is_tty_connected()

        self.assertEqual(result, False)

    @mock.patch("sys.stdin.isatty", return_value=True)
    def test_is_tty_connected(self, tty_mock):
        result = echo.is_tty_connected()

        self.assertEqual(result, True)

    @mock.patch("snapcraft.cli.echo.is_tty_connected", return_value=False)
    def test_echo_confirm_is_not_tty(self, tty_mock):
        echo.confirm("message")

        self.click_confirm.mock.assert_not_called()

    @mock.patch("snapcraft.cli.echo.is_tty_connected", return_value=True)
    def test_echo_confirm_is_tty(self, tty_mock):
        echo.confirm("message")

        self.click_confirm.mock.assert_called_once_with(
            "message",
            default=False,
            abort=False,
            prompt_suffix=": ",
            show_default=True,
            err=False,
        )

    @mock.patch("snapcraft.cli.echo.is_tty_connected", return_value=True)
    def test_echo_confirm_default(self, tty_mock):
        echo.confirm("message", default="the new default")

        self.click_confirm.mock.assert_called_once_with(
            "message",
            default="the new default",
            abort=False,
            prompt_suffix=": ",
            show_default=True,
            err=False,
        )

    @mock.patch("snapcraft.cli.echo.is_tty_connected", return_value=False)
    def test_echo_prompt_is_not_tty(self, tty_mock):
        echo.prompt("message")

        self.click_prompt.mock.assert_not_called()

    @mock.patch("snapcraft.cli.echo.is_tty_connected", return_value=True)
    def test_echo_prompt_is_tty(self, tty_mock):
        echo.prompt("message")

        self.click_prompt.mock.assert_called_once_with(
            "message",
            default=None,
            hide_input=False,
            confirmation_prompt=False,
            type=None,
            value_proc=None,
            prompt_suffix=": ",
            show_default=True,
            err=False,
        )

    @mock.patch("snapcraft.cli.echo.is_tty_connected", return_value=True)
    def test_echo_prompt_default(self, tty_mock):
        echo.prompt("message", default="the new default")

        self.click_prompt.mock.assert_called_once_with(
            "message",
            default="the new default",
            hide_input=False,
            confirmation_prompt=False,
            type=None,
            value_proc=None,
            prompt_suffix=": ",
            show_default=True,
            err=False,
        )


def test_exit_error_required_kwargs(mock_echo_error, mock_sys_exit):
    echo.exit_error(brief="You failed!", resolution="Try again!")

    mock_echo_error.assert_called_once_with(
        dedent(
            """\
    You failed!

    Recommended resolution:
    Try again!"""
        )
    )
    mock_sys_exit.assert_called_once_with(2)


def test_exit_error_all_kwargs(mock_echo_error, mock_sys_exit):
    echo.exit_error(
        brief="You failed!",
        resolution="Try again!",
        details="Try harder!",
        docs_url="https://snapcraft.io/failure",
        exit_code=5,
    )

    mock_echo_error.assert_called_once_with(
        dedent(
            """\
    You failed!

    Recommended resolution:
    Try again!

    Detailed information:
    Try harder!

    For more information, check out:
    https://snapcraft.io/failure"""
        )
    )
    mock_sys_exit.assert_called_once_with(5)


def test_exit_error_all_kwargs_except_details(mock_echo_error, mock_sys_exit):
    echo.exit_error(
        brief="You failed!",
        resolution="Try again!",
        docs_url="https://snapcraft.io/failure",
        exit_code=255,
    )

    mock_echo_error.assert_called_once_with(
        dedent(
            """\
    You failed!

    Recommended resolution:
    Try again!

    For more information, check out:
    https://snapcraft.io/failure"""
        )
    )
    mock_sys_exit.assert_called_once_with(255)


def test_exit_error_all_kwargs_except_docs_url(mock_echo_error, mock_sys_exit):
    echo.exit_error(
        brief="You failed!", resolution="Try again!", details="Try harder!", exit_code=7
    )

    mock_echo_error.assert_called_once_with(
        dedent(
            """\
    You failed!

    Recommended resolution:
    Try again!

    Detailed information:
    Try harder!"""
        )
    )
    mock_sys_exit.assert_called_once_with(7)
