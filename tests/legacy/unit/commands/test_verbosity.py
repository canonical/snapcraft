# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2023 Canonical Ltd
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
import sys

import pytest

from snapcraft_legacy.cli._runner import run


@pytest.fixture
def mock_configure(mocker):
    yield mocker.patch("snapcraft_legacy.internal.log.configure")


@pytest.fixture
def mock_runner(mocker):
    """Mock commands run by the snapcraft_legacy."""
    mocker.patch("snapcraft_legacy.cli.lifecycle._execute")
    mocker.patch("snapcraft_legacy.cli._runner.configure_requests_ca")


@pytest.mark.parametrize("command", ["pull", "build", "stage", "prime", "snap"])
def test_verbosity_debug(command, mock_configure, mock_runner, mocker):
    """`--enable-developer-debug` should set the verbosity."""
    mocker.patch.object(sys, "argv", [command, "--enable-developer-debug"])
    with pytest.raises(SystemExit) as raised:
        run()

    assert raised.value.code == 0
    mock_configure.assert_called_once_with(log_level=logging.DEBUG)


@pytest.mark.parametrize("command", ["pull", "build", "stage", "prime", "snap"])
def test_verbosity_verbose_long(command, mock_configure, mock_runner, mocker):
    """`--verbose` should set the verbosity."""
    mocker.patch.object(sys, "argv", ["pull", "--verbose"])
    with pytest.raises(SystemExit) as raised:
        run()

    assert raised.value.code == 0
    mock_configure.assert_called_once_with(log_level=logging.INFO)


@pytest.mark.parametrize("command", ["pull", "build", "stage", "prime", "snap"])
def test_verbosity_verbose_long_before_command(
    command, mock_configure, mock_runner, mocker
):
    """`--verbose` can precede the command."""
    mocker.patch.object(sys, "argv", [command, "--verbose"])
    with pytest.raises(SystemExit) as raised:
        run()

    assert raised.value.code == 0
    mock_configure.assert_called_once_with(log_level=logging.INFO)


@pytest.mark.parametrize("command", ["pull", "build", "stage", "prime", "snap"])
def test_verbosity_verbose_short(command, mock_configure, mock_runner, mocker):
    """`-v` should set the verbosity."""
    mocker.patch.object(sys, "argv", [command, "-v"])
    with pytest.raises(SystemExit) as raised:
        run()

    assert raised.value.code == 0
    mock_configure.assert_called_once_with(log_level=logging.INFO)


@pytest.mark.parametrize("command", ["pull", "build", "stage", "prime", "snap"])
def test_verbosity_verbose_short_before_command(
    command, mock_configure, mock_runner, mocker
):
    """`-v` can precede the command."""
    mocker.patch.object(sys, "argv", [command, "-v"])
    with pytest.raises(SystemExit) as raised:
        run()

    assert raised.value.code == 0
    mock_configure.assert_called_once_with(log_level=logging.INFO)


@pytest.mark.parametrize("command", ["pull", "build", "stage", "prime", "snap"])
def test_verbosity_verbose_long_and_debug_error(
    capsys, command, mock_configure, mock_runner, mocker
):
    mocker.patch.object(sys, "argv", [command, "--verbose", "--enable-developer-debug"])
    with pytest.raises(SystemExit) as raised:
        run()

    assert raised.value.code == 2
    assert (
        "Error: The 'enable-developer-debug' and 'verbose' options are mutually exclusive."
        in capsys.readouterr().err
    )


@pytest.mark.parametrize("command", ["pull", "build", "stage", "prime", "snap"])
def test_verbosity_verbose_short_and_debug_error(
    capsys, command, mock_configure, mock_runner, mocker
):
    mocker.patch.object(sys, "argv", [command, "-v", "--enable-developer-debug"])
    with pytest.raises(SystemExit) as raised:
        run()

    assert raised.value.code == 2
    assert (
        "Error: The 'enable-developer-debug' and 'verbose' options are mutually exclusive."
        in capsys.readouterr().err
    )
