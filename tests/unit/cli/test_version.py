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

import argparse
import sys
from unittest.mock import call

import pytest

from snapcraft import __version__, cli


def test_version_command(mocker):
    mocker.patch.object(sys, "argv", ["cmd", "version"])
    mock_version_cmd = mocker.patch("snapcraft.commands.version.VersionCommand.run")
    cli.run()
    assert mock_version_cmd.mock_calls == [call(argparse.Namespace())]


def test_version_argument(mocker, capsys):
    mocker.patch.object(sys, "argv", ["cmd", "--version"])
    # FIXME: handled by legacy, change after craft-cli handles default command
    with pytest.raises(SystemExit):
        cli.run()
    assert capsys.readouterr().out == f"snapcraft {__version__}\n"


def test_version_argument_with_command(mocker, emitter):
    mocker.patch.object(sys, "argv", ["cmd", "--version", "version"])
    cli.run()
    emitter.assert_recorded([f"snapcraft {__version__}"])
