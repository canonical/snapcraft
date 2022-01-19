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

import sys

import pytest

from snapcraft import cli


def test_version_command(mocker):
    mocker.patch.object(sys, "argv", ["cmd", "version"])
    mock_version_cmd = mocker.patch("snapcraft.commands.VersionCommand")
    cli.run()
    assert mock_version_cmd.mock_calls == []


def test_version_argument(mocker):
    mocker.patch.object(sys, "argv", ["cmd", "--version"])
    # FIXME: handled by legacy, change after craft-cli handles default command
    mock_version_cmd = mocker.patch("snapcraft_legacy.cli.version.version")
    with pytest.raises(SystemExit):
        cli.run()
    assert mock_version_cmd.mock_calls == []


def test_version_argment_with_command(mocker):
    mocker.patch.object(sys, "argv", ["cmd", "--version", "version"])
    mock_version_cmd = mocker.patch("snapcraft.commands.VersionCommand")
    cli.run()
    assert mock_version_cmd.mock_calls == []
