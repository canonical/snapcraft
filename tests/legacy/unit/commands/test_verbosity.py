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
from snapcraft_legacy.internal import errors


@pytest.fixture
def mock_configure(mocker):
    yield mocker.patch("snapcraft_legacy.internal.log.configure")


@pytest.fixture(autouse=True)
def mock_runner(mocker):
    """Mock commands run by the snapcraft_legacy."""
    mocker.patch("snapcraft_legacy.cli.lifecycle._execute")
    mocker.patch("snapcraft_legacy.cli._runner.configure_requests_ca")


@pytest.mark.parametrize(
    "argument",
    ["-v", "--verbose", "--quiet", "--verbosity=brief"],
)
@pytest.mark.parametrize("command", ["pull", "build", "stage", "prime", "snap"])
@pytest.mark.parametrize("flip_order", [True, False])
def test_ignore_verbosity_args(argument, command, flip_order, mock_configure, mocker, monkeypatch):
    """Command line args don't determine the verbosity.

    Modern Snapcraft passes the verbosity with CRAFT_VERBOSITY_LEVEL, so legacy snapcraft
    ignores command line args that set verbosity and instead uses the env var.

    The exception is `--enable-developer-debug`, which is still used.
    """
    monkeypatch.setenv("CRAFT_VERBOSITY_LEVEL", "TRACE")

    args = [command, argument] if flip_order else [argument, command]

    mocker.patch.object(sys, "argv", args)
    with pytest.raises(SystemExit) as raised:
        run()

    assert raised.value.code == 0
    # use the verbosity set in CRAFT_VERBOSITY_LEVEL
    mock_configure.assert_called_once_with(log_level=logging.DEBUG)


@pytest.mark.parametrize("command", ["pull", "build", "stage", "prime", "snap"])
def test_verbosity_enable_developer_debug(command, mock_configure, mocker, monkeypatch):
    """`--enable-developer-debug` sets the verbosity."""
    monkeypatch.delenv("CRAFT_VERBOSITY_LEVEL", raising=False)
    mocker.patch.object(sys, "argv", [command, "--enable-developer-debug"])

    with pytest.raises(SystemExit) as raised:
        run()

    assert raised.value.code == 0
    mock_configure.assert_called_once_with(log_level=logging.DEBUG)


@pytest.mark.parametrize(
    ["verbosity", "level"],
    [
        ["quiet", logging.CRITICAL],
        ["QUIET", logging.CRITICAL],
        [None, logging.INFO],
        ["brief", logging.INFO],
        ["BRIEF", logging.INFO],
        ["verbose", logging.INFO],
        ["VERBOSE", logging.INFO],
        ["debug", logging.DEBUG],
        ["DEBUG", logging.DEBUG],
        ["trace", logging.DEBUG],
        ["TRACE", logging.DEBUG],
    ]
)
@pytest.mark.parametrize("command", ["pull", "build", "stage", "prime", "snap"])
def test_verbosity_env(command, verbosity, level, mock_configure, mocker, monkeypatch):
    """`CRAFT_VERBOSITY_LEVEL` sets the verbosity."""
    if verbosity:
        monkeypatch.setenv("CRAFT_VERBOSITY_LEVEL", verbosity)
    else:
        monkeypatch.delenv("CRAFT_VERBOSITY_LEVEL", raising=False)
    mocker.patch.object(sys, "argv", [command])

    with pytest.raises(SystemExit) as raised:
        run()

    assert raised.value.code == 0
    mock_configure.assert_called_once_with(log_level=level)


@pytest.mark.parametrize("command", ["pull", "build", "stage", "prime", "snap"])
def test_verbosity_env_error(command, mock_configure, mocker, monkeypatch):
    """Error if `CRAFT_VERBOSITY_LEVEL` is invalid."""
    monkeypatch.setenv("CRAFT_VERBOSITY_LEVEL", "UNKNOWN")
    mocker.patch.object(sys, "argv", [command])

    with pytest.raises(errors.SnapcraftVerbosityError) as raised:
        run()

    assert str(raised.value) == (
        "Invalid verbosity level 'UNKNOWN'. Valid levels are: QUIET, BRIEF, VERBOSE, DEBUG, TRACE."
    )


@pytest.mark.parametrize(
    "arguments",
    [
        ["-v", "--enable-developer-debug"],
        ["--verbose", "--enable-developer-debug"],
        ["--quiet", "--enable-developer-debug"],
        ["--quiet", "--verbosity", "trace"],
    ]
)
@pytest.mark.parametrize("command", ["pull", "build", "stage", "prime", "snap"])
def test_verbosity_mutually_exclusive_error(
    capsys, command, arguments, mock_configure, mocker
):
    """Error when mutually exclusive arguments are provided."""
    mocker.patch.object(sys, "argv", [command, *arguments])
    with pytest.raises(SystemExit) as raised:
        run()

    assert raised.value.code == 2
    assert (
        "The 'verbose', 'quiet', 'verbosity', and 'enable-developer-debug' "
        "options are mutually exclusive."
        in capsys.readouterr().err
    )
