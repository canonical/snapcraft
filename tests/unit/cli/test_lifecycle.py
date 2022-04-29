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
    assert mock_lifecycle_cmd.mock_calls == [
        call(
            argparse.Namespace(
                parts=[],
                debug=False,
                destructive_mode=False,
                shell=False,
                shell_after=False,
                use_lxd=False,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
            )
        )
    ]


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
    mocker.patch.object(
        sys,
        "argv",
        [
            "cmd",
            cmd,
            "part1",
            "part2",
        ],
    )
    mock_lifecycle_cmd = mocker.patch(run_method)
    cli.run()
    assert mock_lifecycle_cmd.mock_calls == [
        call(
            argparse.Namespace(
                parts=["part1", "part2"],
                debug=False,
                destructive_mode=False,
                shell=False,
                shell_after=False,
                use_lxd=False,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
            )
        )
    ]


@pytest.mark.parametrize(
    "cmd,run_method",
    [
        ("pull", "snapcraft.commands.lifecycle.PullCommand.run"),
        ("build", "snapcraft.commands.lifecycle.BuildCommand.run"),
        ("stage", "snapcraft.commands.lifecycle.StageCommand.run"),
        ("prime", "snapcraft.commands.lifecycle.PrimeCommand.run"),
    ],
)
def test_lifecycle_command_arguments_destructive_mode(cmd, run_method, mocker):
    mocker.patch.object(
        sys,
        "argv",
        [
            "cmd",
            cmd,
            "--destructive-mode",
            "part1",
            "part2",
        ],
    )
    mock_lifecycle_cmd = mocker.patch(run_method)
    cli.run()
    assert mock_lifecycle_cmd.mock_calls == [
        call(
            argparse.Namespace(
                parts=["part1", "part2"],
                debug=False,
                destructive_mode=True,
                shell=False,
                shell_after=False,
                use_lxd=False,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
            )
        )
    ]


@pytest.mark.parametrize(
    "cmd,run_method",
    [
        ("pull", "snapcraft.commands.lifecycle.PullCommand.run"),
        ("build", "snapcraft.commands.lifecycle.BuildCommand.run"),
        ("stage", "snapcraft.commands.lifecycle.StageCommand.run"),
        ("prime", "snapcraft.commands.lifecycle.PrimeCommand.run"),
    ],
)
def test_lifecycle_command_arguments_use_lxd(cmd, run_method, mocker):
    mocker.patch.object(
        sys,
        "argv",
        [
            "cmd",
            cmd,
            "--use-lxd",
            "part1",
            "part2",
        ],
    )
    mock_lifecycle_cmd = mocker.patch(run_method)
    cli.run()
    assert mock_lifecycle_cmd.mock_calls == [
        call(
            argparse.Namespace(
                parts=["part1", "part2"],
                debug=False,
                destructive_mode=False,
                shell=False,
                shell_after=False,
                use_lxd=True,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
            )
        )
    ]


@pytest.mark.parametrize(
    "cmd,run_method",
    [
        ("pull", "snapcraft.commands.lifecycle.PullCommand.run"),
        ("build", "snapcraft.commands.lifecycle.BuildCommand.run"),
        ("stage", "snapcraft.commands.lifecycle.StageCommand.run"),
        ("prime", "snapcraft.commands.lifecycle.PrimeCommand.run"),
    ],
)
def test_lifecycle_command_arguments_debug(cmd, run_method, mocker):
    mocker.patch.object(
        sys,
        "argv",
        [
            "cmd",
            cmd,
            "--debug",
        ],
    )
    mock_lifecycle_cmd = mocker.patch(run_method)
    cli.run()
    assert mock_lifecycle_cmd.mock_calls == [
        call(
            argparse.Namespace(
                parts=[],
                debug=True,
                destructive_mode=False,
                shell=False,
                shell_after=False,
                use_lxd=False,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
            )
        )
    ]


@pytest.mark.parametrize(
    "cmd,run_method",
    [
        ("pull", "snapcraft.commands.lifecycle.PullCommand.run"),
        ("build", "snapcraft.commands.lifecycle.BuildCommand.run"),
        ("stage", "snapcraft.commands.lifecycle.StageCommand.run"),
        ("prime", "snapcraft.commands.lifecycle.PrimeCommand.run"),
    ],
)
def test_lifecycle_command_arguments_shell(cmd, run_method, mocker):
    mocker.patch.object(
        sys,
        "argv",
        [
            "cmd",
            cmd,
            "--shell",
        ],
    )
    mock_lifecycle_cmd = mocker.patch(run_method)
    cli.run()
    assert mock_lifecycle_cmd.mock_calls == [
        call(
            argparse.Namespace(
                parts=[],
                debug=False,
                destructive_mode=False,
                shell=True,
                shell_after=False,
                use_lxd=False,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
            )
        )
    ]


@pytest.mark.parametrize(
    "cmd,run_method",
    [
        ("pull", "snapcraft.commands.lifecycle.PullCommand.run"),
        ("build", "snapcraft.commands.lifecycle.BuildCommand.run"),
        ("stage", "snapcraft.commands.lifecycle.StageCommand.run"),
        ("prime", "snapcraft.commands.lifecycle.PrimeCommand.run"),
    ],
)
def test_lifecycle_command_arguments_shell_after(cmd, run_method, mocker):
    mocker.patch.object(
        sys,
        "argv",
        [
            "cmd",
            cmd,
            "--shell-after",
        ],
    )
    mock_lifecycle_cmd = mocker.patch(run_method)
    cli.run()
    assert mock_lifecycle_cmd.mock_calls == [
        call(
            argparse.Namespace(
                parts=[],
                debug=False,
                destructive_mode=False,
                shell=False,
                shell_after=True,
                use_lxd=False,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
            )
        )
    ]


def test_lifecycle_command_pack(mocker):
    mocker.patch.object(
        sys,
        "argv",
        ["cmd", "pack"],
    )
    mock_pack_cmd = mocker.patch("snapcraft.commands.lifecycle.PackCommand.run")
    cli.run()
    assert mock_pack_cmd.mock_calls == [
        call(
            argparse.Namespace(
                directory=None,
                output=None,
                debug=False,
                destructive_mode=False,
                use_lxd=False,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
            )
        )
    ]


def test_lifecycle_command_pack_destructive_mode(mocker):
    mocker.patch.object(
        sys,
        "argv",
        ["cmd", "pack", "--destructive-mode"],
    )
    mock_pack_cmd = mocker.patch("snapcraft.commands.lifecycle.PackCommand.run")
    cli.run()
    assert mock_pack_cmd.mock_calls == [
        call(
            argparse.Namespace(
                directory=None,
                output=None,
                debug=False,
                destructive_mode=True,
                use_lxd=False,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
            )
        )
    ]


def test_lifecycle_command_pack_use_lxd(mocker):
    mocker.patch.object(
        sys,
        "argv",
        ["cmd", "pack", "--use-lxd"],
    )
    mock_pack_cmd = mocker.patch("snapcraft.commands.lifecycle.PackCommand.run")
    cli.run()
    assert mock_pack_cmd.mock_calls == [
        call(
            argparse.Namespace(
                directory=None,
                output=None,
                debug=False,
                destructive_mode=False,
                use_lxd=True,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
            )
        )
    ]


def test_lifecycle_command_pack_debug(mocker):
    mocker.patch.object(
        sys,
        "argv",
        ["cmd", "pack", "--debug"],
    )
    mock_pack_cmd = mocker.patch("snapcraft.commands.lifecycle.PackCommand.run")
    cli.run()
    assert mock_pack_cmd.mock_calls == [
        call(
            argparse.Namespace(
                directory=None,
                output=None,
                debug=True,
                destructive_mode=False,
                use_lxd=False,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
            )
        )
    ]


@pytest.mark.parametrize("option", ["-o", "--output"])
def test_lifecycle_command_pack_output(mocker, option):
    mocker.patch.object(sys, "argv", ["cmd", "pack", option, "name"])
    mock_pack_cmd = mocker.patch("snapcraft.commands.lifecycle.PackCommand.run")
    cli.run()
    assert mock_pack_cmd.mock_calls == [
        call(
            argparse.Namespace(
                directory=None,
                output="name",
                debug=False,
                destructive_mode=False,
                use_lxd=False,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
            )
        )
    ]


def test_lifecycle_command_pack_directory(mocker):
    mocker.patch.object(sys, "argv", ["cmd", "pack", "name"])
    mock_pack_cmd = mocker.patch("snapcraft.commands.lifecycle.PackCommand.run")
    cli.run()
    assert mock_pack_cmd.mock_calls == [
        call(
            argparse.Namespace(
                debug=False,
                destructive_mode=False,
                directory="name",
                output=None,
                use_lxd=False,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
            )
        )
    ]
