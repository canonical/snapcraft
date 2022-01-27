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

from snapcraft import cli


@pytest.mark.parametrize(
    "cmd,run_method",
    [
        ("pull", "snapcraft.commands.lifecycle.PullCommand.run"),
        ("build", "snapcraft.commands.lifecycle.BuildCommand.run"),
        ("stage", "snapcraft.commands.lifecycle.StageCommand.run"),
        ("prime", "snapcraft.commands.lifecycle.PrimeCommand.run"),
    ],
)
def test_lifecycle_command(cmd, run_method, mocker):
    mocker.patch.object(sys, "argv", ["cmd", cmd])
    mock_lifecycle_cmd = mocker.patch(run_method)
    cli.run()
    assert mock_lifecycle_cmd.mock_calls == [call(argparse.Namespace(parts=[]))]


@pytest.mark.parametrize(
    "cmd,run_method",
    [
        ("pull", "snapcraft.commands.lifecycle.PullCommand.run"),
        ("build", "snapcraft.commands.lifecycle.BuildCommand.run"),
        ("stage", "snapcraft.commands.lifecycle.StageCommand.run"),
        ("prime", "snapcraft.commands.lifecycle.PrimeCommand.run"),
    ],
)
def test_lifecycle_command_arguments(cmd, run_method, mocker):
    mocker.patch.object(sys, "argv", ["cmd", cmd, "part1", "part2"])
    mock_lifecycle_cmd = mocker.patch(run_method)
    cli.run()
    assert mock_lifecycle_cmd.mock_calls == [
        call(argparse.Namespace(parts=["part1", "part2"]))
    ]
