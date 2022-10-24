# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

import pytest
from craft_cli import ArgumentParsingError

from snapcraft import cli


@pytest.mark.parametrize("value", ["yes", "YES", "1", "y", "Y"])
def test_developer_debug_in_env_sets_debug(monkeypatch, value):
    """DEVELOPER_DEBUG sets verbosity level to 'debug'."""
    monkeypatch.setenv("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG", value)

    assert cli.get_verbosity() == cli.EmitterMode.DEBUG


@pytest.mark.parametrize("value", ["no", "NO", "0", "n", "N"])
def test_developer_debug_in_env_sets_brief(monkeypatch, value):
    """Default verbosity is used when DEVELOPER_DEBUG=0."""
    monkeypatch.setenv("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG", value)
    monkeypatch.setattr("sys.stdin.isatty", lambda: True)

    assert cli.get_verbosity() == cli.EmitterMode.BRIEF


@pytest.mark.parametrize("value", ["foo", "BAR", "2"])
def test_parse_issue_in_developer_debug_in_env_sets_brief(monkeypatch, value):
    """Invalid values for DEVELOPER_DEBUG are silently ignored."""
    monkeypatch.setenv("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG", value)
    monkeypatch.setattr("sys.stdin.isatty", lambda: True)

    assert cli.get_verbosity() == cli.EmitterMode.BRIEF


def test_no_env_returns_brief(monkeypatch):
    """Default verbosity is used when there is no value for DEVELOPER_DEBUG."""
    monkeypatch.delenv("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG", False)
    monkeypatch.setattr("sys.stdin.isatty", lambda: True)

    assert cli.get_verbosity() == cli.EmitterMode.BRIEF


def test_sdtin_no_tty_returns_verbose(monkeypatch):
    """Default verbosity for environments with a closed stdin is used when there is no
    value for DEVELOPER_DEBUG."""
    monkeypatch.delenv("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG", False)
    monkeypatch.setattr("sys.stdin.isatty", lambda: False)

    assert cli.get_verbosity() == cli.EmitterMode.VERBOSE


def test_developer_debug_and_sdtin_no_tty_returns_debug(monkeypatch):
    """DEVELOPER_DEBUG sets verbosity in an environment with a closed stdin."""
    monkeypatch.setenv("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG", "y")
    monkeypatch.setattr("sys.stdin.isatty", lambda: False)

    assert cli.get_verbosity() == cli.EmitterMode.DEBUG


@pytest.mark.parametrize("developer_debug", ["n", "y"])
@pytest.mark.parametrize("isatty", [False, True])
@pytest.mark.parametrize("verbosity", ["quiet", "brief", "verbose", "debug", "trace"])
def test_env_var(monkeypatch, developer_debug, isatty, verbosity):
    """Environment variable sets verbosity, even when DEVELOPER_DEBUG is defined or
    stdin is closed."""
    monkeypatch.setenv("SNAPCRAFT_VERBOSITY_LEVEL", verbosity)
    monkeypatch.setenv("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG", developer_debug)
    monkeypatch.setattr("sys.stdin.isatty", lambda: isatty)

    assert cli.get_verbosity() == cli.EmitterMode[verbosity.upper()]


def test_env_var_invalid(monkeypatch):
    """Error is raised when environmental variable is invalid."""
    monkeypatch.setenv("SNAPCRAFT_VERBOSITY_LEVEL", "invalid")

    with pytest.raises(ArgumentParsingError) as raised:
        cli.get_verbosity()

    assert str(raised.value) == (
        "cannot parse verbosity level 'invalid' from environmental "
        "variable SNAPCRAFT_VERBOSITY_LEVEL (valid values are 'quiet', 'brief', "
        "'verbose', 'debug' and 'trace')"
    )
