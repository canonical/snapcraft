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
import re
from unittest.mock import call

import pytest

import snapcraft.commands.core22.lifecycle as core22_lifecycle
import snapcraft.errors
from snapcraft.application import APP_METADATA
from snapcraft.commands import lifecycle


@pytest.mark.parametrize(
    "cmd_class",
    [
        core22_lifecycle.PullCommand,
        core22_lifecycle.BuildCommand,
        core22_lifecycle.StageCommand,
        core22_lifecycle.PrimeCommand,
        core22_lifecycle.CleanCommand,
        core22_lifecycle.TryCommand,
    ],
)
def test_core22_lifecycle_command(cmd_class, mocker):
    lifecycle_run_mock = mocker.patch("snapcraft.parts.lifecycle.run")
    cmd = cmd_class(None)

    cmd.run(argparse.Namespace(parts=["part1", "part2"]))

    assert lifecycle_run_mock.mock_calls == [
        call(cmd_class.name, argparse.Namespace(parts=["part1", "part2"]))
    ]


def test_core22_pack_command(mocker, emitter):
    lifecycle_run_mock = mocker.patch("snapcraft.parts.lifecycle.run")
    cmd = core22_lifecycle.PackCommand(None)

    cmd.run(argparse.Namespace(directory=None, output=None, compression=None))

    assert lifecycle_run_mock.mock_calls == [
        call(
            "pack",
            argparse.Namespace(directory=None, output=None, compression=None),
        )
    ]


def test_core22_pack_command_with_output(mocker, emitter):
    lifecycle_run_mock = mocker.patch("snapcraft.parts.lifecycle.run")
    pack_mock = mocker.patch("snapcraft.pack.pack_snap")
    cmd = core22_lifecycle.PackCommand(None)

    cmd.run(argparse.Namespace(directory=None, output="output", compression=None))

    assert lifecycle_run_mock.mock_calls == [
        call(
            "pack",
            argparse.Namespace(compression=None, directory=None, output="output"),
        )
    ]
    assert pack_mock.mock_calls == []


def test_core22_pack_command_with_directory(mocker):
    lifecycle_run_mock = mocker.patch("snapcraft.parts.lifecycle.run")
    pack_mock = mocker.patch("snapcraft.pack.pack_snap")
    cmd = core22_lifecycle.PackCommand(None)

    cmd.run(argparse.Namespace(directory=".", output=None, compression=None))

    assert lifecycle_run_mock.mock_calls == []
    assert pack_mock.mock_calls[0] == call(".", output=None)


def test_snap_command_error(mocker):
    """Error on the removed core22 'snap' command."""
    cmd = core22_lifecycle.SnapCommand(None)
    expected = re.escape("The 'snap' command was renamed to 'pack'.")

    with pytest.raises(snapcraft.errors.RemovedCommand, match=expected):
        cmd.run(argparse.Namespace(directory=None, output=None, compression=None))


@pytest.mark.usefixtures("emitter")
def test_core24_try_command(tmp_path, fake_services):
    parsed_args = argparse.Namespace(parts=[], output=tmp_path)
    cmd = lifecycle.TryCommand({"app": APP_METADATA, "services": fake_services})

    with pytest.raises(snapcraft.errors.FeatureNotImplemented) as raised:
        cmd.run(parsed_args=parsed_args)

    assert str(raised.value) == (
        'Command or feature not implemented: "snapcraft try" is not '
        "implemented for core24"
    )


def test_core24_pack(mocker, emitter, fake_services, tmp_path):
    parsed_args = argparse.Namespace(
        destructive_mode=False, directory=tmp_path, output="test-output"
    )
    pack_mock = mocker.patch("snapcraft.pack.pack_snap")
    cmd = lifecycle.PackCommand({"app": APP_METADATA, "services": fake_services})

    cmd.run(parsed_args)

    assert pack_mock.mock_calls[0] == call(tmp_path, output="test-output")


def test_core24_snap_error(fake_services, tmp_path):
    """Error on the removed core24 'snap' command."""
    parsed_args = argparse.Namespace(
        destructive_mode=False, directory=tmp_path, output="test-output"
    )
    cmd = lifecycle.SnapCommand({"app": APP_METADATA, "services": fake_services})
    expected = re.escape("The 'snap' command was renamed to 'pack'.")

    with pytest.raises(snapcraft.errors.RemovedCommand, match=expected):
        cmd.run(parsed_args)
