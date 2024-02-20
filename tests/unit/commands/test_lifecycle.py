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
from unittest.mock import call

import pytest

from snapcraft.commands.core22.lifecycle import (
    BuildCommand,
    CleanCommand,
    PackCommand,
    PrimeCommand,
    PullCommand,
    SnapCommand,
    StageCommand,
    TryCommand,
)


@pytest.mark.parametrize(
    "cmd_name,cmd_class",
    [
        ("pull", PullCommand),
        ("build", BuildCommand),
        ("stage", StageCommand),
        ("prime", PrimeCommand),
        ("clean", CleanCommand),
        ("try", TryCommand),
    ],
)
def test_lifecycle_command(cmd_name, cmd_class, mocker):
    lifecycle_run_mock = mocker.patch("snapcraft.parts.lifecycle.run")
    cmd = cmd_class(None)
    cmd.run(argparse.Namespace(parts=["part1", "part2"]))
    assert lifecycle_run_mock.mock_calls == [
        call(cmd_name, argparse.Namespace(parts=["part1", "part2"]))
    ]


@pytest.mark.parametrize(
    "cmd_name,cmd_class",
    [
        ("pack", PackCommand),
        ("snap", SnapCommand),
    ],
)
def test_pack_command(mocker, cmd_name, cmd_class):
    lifecycle_run_mock = mocker.patch("snapcraft.parts.lifecycle.run")
    cmd = cmd_class(None)
    cmd.run(argparse.Namespace(directory=None, output=None, compression=None))
    assert lifecycle_run_mock.mock_calls == [
        call(
            cmd_name, argparse.Namespace(directory=None, output=None, compression=None)
        )
    ]


@pytest.mark.parametrize(
    "cmd_name,cmd_class",
    [
        ("pack", PackCommand),
        ("snap", SnapCommand),
    ],
)
def test_pack_command_with_output(mocker, cmd_name, cmd_class):
    lifecycle_run_mock = mocker.patch("snapcraft.parts.lifecycle.run")
    pack_mock = mocker.patch("snapcraft.pack.pack_snap")
    cmd = cmd_class(None)
    cmd.run(argparse.Namespace(directory=None, output="output", compression=None))
    assert lifecycle_run_mock.mock_calls == [
        call(
            cmd_name,
            argparse.Namespace(compression=None, directory=None, output="output"),
        )
    ]
    assert pack_mock.mock_calls == []


def test_pack_command_with_directory(mocker):
    lifecycle_run_mock = mocker.patch("snapcraft.parts.lifecycle.run")
    pack_mock = mocker.patch("snapcraft.pack.pack_snap")
    cmd = PackCommand(None)
    cmd.run(argparse.Namespace(directory=".", output=None, compression=None))
    assert lifecycle_run_mock.mock_calls == []
    assert pack_mock.mock_calls[0] == call(".", output=None)
