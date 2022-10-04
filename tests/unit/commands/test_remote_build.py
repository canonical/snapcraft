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

from snapcraft import errors
from snapcraft.commands.remote import RemoteBuildCommand


@pytest.fixture()
def legacy_run_mock(mocker):
    mock = mocker.patch("snapcraft.commands.remote.run_legacy")
    return mock


@pytest.fixture(autouse=True)
def get_project_mock(mocker):
    mocker.patch("snapcraft.commands.remote.get_snap_project")
    return mocker.patch("snapcraft.commands.remote.process_yaml")


@pytest.fixture
def confirm_mock(mocker):
    return mocker.patch(
        "snapcraft.commands.remote.confirm_with_user", return_value=True
    )


@pytest.fixture
def fake_sudo(monkeypatch):
    monkeypatch.setenv("SUDO_USER", "fake")
    monkeypatch.setattr("os.geteuid", lambda: 0)


def test_command_accept_upload_runs_legacy(legacy_run_mock, emitter):
    cmd = RemoteBuildCommand(None)
    cmd.run(
        argparse.Namespace(
            status=None,
            recover=None,
            build_for=None,
            build_on=None,
            launchpad_accept_public_upload=True,
        )
    )

    assert legacy_run_mock.mock_calls == [call()]
    emitter.assert_message(
        "snapcraft remote-build is experimental and is subject to change "
        "- "
        "use with caution."
    )
    emitter.assert_debug(
        "core22 not yet supported in new code base: re-executing into legacy for remote-build"
    )


def test_command_user_confirms_upload_runs_legacy(
    legacy_run_mock, emitter, confirm_mock
):
    cmd = RemoteBuildCommand(None)
    cmd.run(
        argparse.Namespace(
            status=None,
            recover=None,
            build_for=None,
            build_on=None,
            launchpad_accept_public_upload=None,
        )
    )

    assert legacy_run_mock.mock_calls == [call()]
    assert confirm_mock.mock_calls == [
        call(
            "All data sent to remote builders will be publicly available. "
            "Are you sure you want to continue?"
        )
    ]
    emitter.assert_message(
        "snapcraft remote-build is experimental and is subject to change "
        "- "
        "use with caution."
    )


def test_command_legacy_exec_on_project_fail_after_confirming_with_user(
    emitter, confirm_mock, get_project_mock
):
    """Legacy fallback is raised after confirming with the user.

    In this scenario, Snapcraft's exit handler handles the re-exec.
    """
    get_project_mock.side_effect = errors.LegacyFallback("Unsupported base")
    cmd = RemoteBuildCommand(None)

    with pytest.raises(errors.LegacyFallback):
        cmd.run(
            argparse.Namespace(
                status=None,
                recover=None,
                build_for=None,
                build_on=None,
                launchpad_accept_public_upload=None,
            )
        )

    assert confirm_mock.mock_calls == [
        call(
            "All data sent to remote builders will be publicly available. "
            "Are you sure you want to continue?"
        )
    ]
    emitter.assert_message(
        "snapcraft remote-build is experimental and is subject to change "
        "- "
        "use with caution."
    )


def test_command_use_of_build_on_warns(legacy_run_mock, emitter):
    cmd = RemoteBuildCommand(None)
    cmd.run(
        argparse.Namespace(
            status=None,
            recover=None,
            build_for=None,
            build_on="amd64",
            launchpad_accept_public_upload=True,
        )
    )

    assert legacy_run_mock.mock_calls == [call()]
    emitter.assert_message(
        "snapcraft remote-build is experimental and is subject to change "
        "- "
        "use with caution."
    )
    emitter.assert_debug(
        "core22 not yet supported in new code base: re-executing into legacy for remote-build"
    )
    emitter.assert_message("Use --build-for instead of --build-on")


@pytest.mark.usefixtures("fake_sudo", "legacy_run_mock")
def test_remote_build_sudo_warns(emitter):
    cmd = RemoteBuildCommand(None)
    cmd.run(
        argparse.Namespace(
            status=None,
            recover=None,
            build_for=None,
            build_on="amd64",
            launchpad_accept_public_upload=True,
        )
    )

    emitter.assert_message(
        "Running with 'sudo' may cause permission errors and is discouraged."
    )
