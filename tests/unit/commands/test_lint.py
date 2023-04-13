# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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

import argparse

import pytest
from craft_cli.errors import ArgumentParsingError

from snapcraft.commands import LintCommand


def test_lint_default(emitter, tmp_path):
    """Test the lint command."""
    # create a snap file
    snap_file = tmp_path / "test-snap.snap"
    snap_file.touch()

    LintCommand(None).run(argparse.Namespace(snap_file=str(snap_file)))

    emitter.assert_progress("Running linter.", permanent=True)
    emitter.assert_progress("'snapcraft lint' not implemented.", permanent=True)


def test_lint_default_snap_file_missing(emitter, tmp_path):
    """Raise an error if the snap file does not exist."""
    # do not create a snap file
    snap_file = tmp_path / "test-snap.snap"

    with pytest.raises(ArgumentParsingError) as raised:
        LintCommand(None).run(argparse.Namespace(snap_file=str(snap_file)))

    assert str(raised.value) == f"Snap file {str(snap_file)!r} does not exist."


def test_lint_default_snap_file_not_valid(emitter, tmp_path):
    """Raise an error if the snap file is not valid."""
    # make the snap filepath a directory
    snap_file = tmp_path / "test-snap.snap"
    snap_file.mkdir()

    with pytest.raises(ArgumentParsingError) as raised:
        LintCommand(None).run(argparse.Namespace(snap_file=snap_file))

    assert str(raised.value) == f"Snap file {str(snap_file)!r} is not a valid file."


def test_lint_assert_file(emitter, tmp_path):
    """Return the assertion file for a snap file."""
    snap_file = tmp_path / "test-snap.snap"
    # create an assertion file
    assert_file = tmp_path / "test-snap.assert"
    assert_file.touch()

    result = LintCommand(None)._get_assert_file(snap_file=snap_file)

    assert result == assert_file
    emitter.assert_debug(f"Found assertion file {str(assert_file)!r}.")


def test_lint_assert_missing(emitter, tmp_path):
    """Return None if the assertion file is missing."""
    snap_file = tmp_path / "test-snap.snap"
    # do not create an assertion file
    assert_file = tmp_path / "test-snap.assert"

    result = LintCommand(None)._get_assert_file(snap_file=snap_file)

    assert not result
    emitter.assert_debug(f"Assertion file {str(assert_file)!r} does not exist.")


def test_lint_assert_file_not_valid(emitter, tmp_path):
    """Return None if the assertion file is not valid."""
    snap_file = tmp_path / "test-snap.snap"
    # make the assertion filepath a directory
    assert_file = tmp_path / "test-snap.assert"
    assert_file.mkdir()

    result = LintCommand(None)._get_assert_file(snap_file=snap_file)

    assert not result
    emitter.assert_debug(f"Assertion file {str(assert_file)!r} is not a valid file.")
