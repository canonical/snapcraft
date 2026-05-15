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
from pathlib import Path
from unittest.mock import call

import pytest
from craft_application.errors import PartsLifecycleError

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
@pytest.mark.parametrize("base", ["core24", "core26"])
def test_try_command(tmp_path, fake_services, base, setup_project, default_project):
    parsed_args = argparse.Namespace(parts=[], output=tmp_path)
    cmd = lifecycle.TryCommand({"app": APP_METADATA, "services": fake_services})
    setup_project(
        fake_services, {**default_project.marshal(), "base": base}, write_project=True
    )

    expected = f"Command or feature not implemented: 'snapcraft try' is not implemented for {base!r}"
    with pytest.raises(
        snapcraft.errors.FeatureNotImplemented, match=re.escape(expected)
    ):
        cmd.run(parsed_args=parsed_args)


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


# Reproducer for https://github.com/canonical/snapcraft/issues/6219
# --shell and --debug fail cryptically with late pack failures


def _make_pack_args(**overrides):
    """Build a minimal parsed_args namespace for the pack command."""
    defaults = argparse.Namespace(
        shell=False,
        shell_after=False,
        debug=False,
        destructive_mode=True,
        directory=None,
        output=Path("."),
    )
    for key, value in overrides.items():
        setattr(defaults, key, value)
    return defaults


@pytest.mark.usefixtures("emitter")
def test_pack_late_failure_shell_launches_debug_shell(
    mocker, fake_services, setup_project, default_project
):
    """--shell should launch a debug shell when update_project() fails (issue #6219).

    A "late pack failure" occurs when the lifecycle steps complete successfully but
    the post-prime step (update_project) raises because mandatory project fields like
    'version', 'summary', or 'description' were not set via adopt-info.

    When the user passes --shell, a debug shell should be launched so they can inspect
    the build environment, but currently the exception from _run_post_prime_steps()
    propagates before the shell check in _run_real() is reached.
    """
    setup_project(fake_services, default_project.marshal())
    mocker.patch.object(fake_services.get("lifecycle"), "run")
    mocker.patch.object(
        fake_services.get("package"),
        "update_project",
        side_effect=PartsLifecycleError(
            "Project fields 'version', 'summary', and 'description' were not set."
        ),
    )
    mock_launch_shell = mocker.patch(
        "craft_application.commands.lifecycle._launch_shell"
    )

    cmd = lifecycle.PackCommand({"app": APP_METADATA, "services": fake_services})
    with pytest.raises(PartsLifecycleError):
        cmd.run(_make_pack_args(shell=True))

    # Bug: _launch_shell() should be called when --shell is passed, but the exception
    # from _run_post_prime_steps() propagates before the shell check in _run_real().
    mock_launch_shell.assert_called_once()


@pytest.mark.usefixtures("emitter")
def test_pack_late_failure_debug_launches_debug_shell(
    mocker, fake_services, setup_project, default_project
):
    """--debug should launch a debug shell when update_project() fails (issue #6219).

    A "late pack failure" occurs when the lifecycle steps complete successfully but
    the post-prime step (update_project) raises because mandatory project fields like
    'version', 'summary', or 'description' were not set via adopt-info.

    When the user passes --debug, a debug shell should be launched on failure, but
    currently the exception from _run_post_prime_steps() propagates without ever
    entering the try/except that handles --debug in _run_real().
    """
    setup_project(fake_services, default_project.marshal())
    mocker.patch.object(fake_services.get("lifecycle"), "run")
    mocker.patch.object(
        fake_services.get("package"),
        "update_project",
        side_effect=PartsLifecycleError(
            "Project fields 'version', 'summary', and 'description' were not set."
        ),
    )
    mock_launch_shell = mocker.patch(
        "craft_application.commands.lifecycle._launch_shell"
    )

    cmd = lifecycle.PackCommand({"app": APP_METADATA, "services": fake_services})
    with pytest.raises(PartsLifecycleError):
        cmd.run(_make_pack_args(debug=True))

    # Bug: _launch_shell() should be called when --debug is passed and a failure
    # occurs, but _run_post_prime_steps() raises outside the try/except in _run_real().
    mock_launch_shell.assert_called_once()
