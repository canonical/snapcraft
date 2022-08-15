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

from snapcraft import cli


@pytest.mark.parametrize("value", ["yes", "YES", "1", "y", "Y"])
def test_developer_debug_in_env_sets_debug(monkeypatch, value):
    monkeypatch.setenv("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG", value)

    assert cli.get_verbosity() == cli.EmitterMode.DEBUG


@pytest.mark.parametrize("value", ["no", "NO", "0", "n", "N"])
def test_developer_debug_in_env_sets_brief(monkeypatch, value):
    monkeypatch.setenv("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG", value)
    monkeypatch.setattr("sys.stdin.isatty", lambda: True)

    assert cli.get_verbosity() == cli.EmitterMode.BRIEF


@pytest.mark.parametrize("value", ["foo", "BAR", "2"])
def test_parse_issue_in_developer_debug_in_env_sets_brief(monkeypatch, value):
    monkeypatch.setenv("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG", value)
    monkeypatch.setattr("sys.stdin.isatty", lambda: True)

    assert cli.get_verbosity() == cli.EmitterMode.BRIEF


def test_no_env_returns_brief(monkeypatch):
    monkeypatch.delenv("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG", False)
    monkeypatch.setattr("sys.stdin.isatty", lambda: True)

    assert cli.get_verbosity() == cli.EmitterMode.BRIEF


def test_sdtin_no_tty_returns_verbose(monkeypatch):
    monkeypatch.delenv("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG", False)
    monkeypatch.setattr("sys.stdin.isatty", lambda: False)

    assert cli.get_verbosity() == cli.EmitterMode.VERBOSE


def test_developer_debug_and_sdtin_no_tty_returns_debug(monkeypatch):
    monkeypatch.setenv("SNAPCRAFT_ENABLE_DEVELOPER_DEBUG", True)
    monkeypatch.setattr("sys.stdin.isatty", lambda: False)

    assert cli.get_verbosity() == cli.EmitterMode.DEBUG
