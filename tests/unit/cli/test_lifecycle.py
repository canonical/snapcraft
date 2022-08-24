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
import os
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                build_for=None,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                build_for=None,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                build_for=None,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                build_for=None,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
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
def test_lifecycle_command_arguments_bind_ssh(cmd, run_method, mocker):
    mocker.patch.object(
        sys,
        "argv",
        [
            "cmd",
            cmd,
            "--bind-ssh",
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
                shell_after=False,
                use_lxd=False,
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=True,
                build_for=None,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                build_for=None,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                build_for=None,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                build_for=None,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
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
def test_lifecycle_command_arguments_http_proxy(cmd, run_method, mocker):
    mocker.patch.object(sys, "argv", ["cmd", cmd, "--http-proxy", "test-http"])
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                build_for=None,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy="test-http",
                https_proxy=None,
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
def test_lifecycle_command_arguments_https_proxy(cmd, run_method, mocker):
    mocker.patch.object(sys, "argv", ["cmd", cmd, "--https-proxy", "test-https"])
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                build_for=None,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy="test-https",
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                build_for=None,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                build_for=None,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                build_for=None,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
            )
        )
    ]


def test_lifecycle_command_pack_enable_manifest(mocker):
    mocker.patch.object(
        sys,
        "argv",
        ["cmd", "pack", "--enable-manifest"],
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
                enable_manifest=True,
                manifest_image_information=None,
                enable_experimental_extensions=False,
                bind_ssh=False,
                build_for=None,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
            )
        )
    ]


def test_lifecycle_command_pack_env_enable_manifest(mocker):
    mocker.patch.dict(os.environ, {"SNAPCRAFT_BUILD_INFO": "1"})
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
                enable_manifest=True,
                manifest_image_information=None,
                bind_ssh=False,
                build_for=None,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
            )
        )
    ]


def test_lifecycle_command_pack_manifest_image_information(mocker):
    mocker.patch.object(
        sys,
        "argv",
        ["cmd", "pack", "--manifest-image-information", "{'some-info': true}"],
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
                enable_manifest=False,
                manifest_image_information="{'some-info': true}",
                bind_ssh=False,
                build_for=None,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
            )
        )
    ]


def test_lifecycle_command_pack_env_manifest_image_information(mocker):
    mocker.patch.dict(os.environ, {"SNAPCRAFT_IMAGE_INFO": "{'some-info': true}"})
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
                enable_manifest=False,
                manifest_image_information="{'some-info': true}",
                bind_ssh=False,
                build_for=None,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
            )
        )
    ]


def test_lifecycle_command_pack_bind_ssh(mocker):
    mocker.patch.object(
        sys,
        "argv",
        ["cmd", "pack", "--bind-ssh"],
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=True,
                build_for=None,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
            )
        )
    ]


def test_lifecycle_command_pack_build_for(mocker):
    mocker.patch.object(
        sys,
        "argv",
        ["cmd", "pack", "--build-for", "armhf"],
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                build_for="armhf",
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
            )
        )
    ]


def test_lifecycle_command_http_proxy(mocker):
    pass


def test_lifecycle_command_pack_env_build_for(mocker):
    mocker.patch.dict(os.environ, {"SNAPCRAFT_BUILD_FOR": "armhf"})
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                build_for="armhf",
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                build_for=None,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                build_for=None,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                build_for=None,
                enable_experimental_extensions=False,
                enable_developer_debug=False,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy=None,
            )
        )
    ]


def test_lifecycle_command_pack_http_proxy(mocker):
    mocker.patch.object(sys, "argv", ["cmd", "pack", "--http-proxy", "test-http"])
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                build_for=None,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy="test-http",
                https_proxy=None,
            )
        )
    ]


def test_lifecycle_command_pack_https_proxy(mocker):
    mocker.patch.object(sys, "argv", ["cmd", "pack", "--https-proxy", "test-https"])
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
                enable_manifest=False,
                manifest_image_information=None,
                bind_ssh=False,
                build_for=None,
                enable_experimental_target_arch=False,
                target_arch=None,
                provider=None,
                http_proxy=None,
                https_proxy="test-https",
            )
        )
    ]
