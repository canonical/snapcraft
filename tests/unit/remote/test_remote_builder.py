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

"""Remote builder tests."""

import re
from pathlib import Path
from unittest.mock import call, patch

import pytest

from snapcraft.remote import GitType, RemoteBuilder, UnsupportedArchitectureError
from snapcraft.remote.utils import _SUPPORTED_ARCHS


@pytest.fixture(autouse=True)
def mock_launchpad_client(mocker):
    """Returns a mocked LaunchpadClient."""
    _mock_launchpad_client = mocker.patch(
        "snapcraft.remote.remote_builder.LaunchpadClient"
    )
    _mock_launchpad_client.return_value.has_outstanding_build.return_value = False
    _mock_launchpad_client.return_value.architectures = ["amd64"]
    return _mock_launchpad_client


@pytest.fixture(autouse=True)
def mock_worktree(mocker):
    """Returns a mocked WorkTree."""
    return mocker.patch("snapcraft.remote.remote_builder.WorkTree")


@pytest.fixture()
def fake_remote_builder(new_dir, mock_launchpad_client, mock_worktree):
    """Returns a fake RemoteBuilder."""
    return RemoteBuilder(
        app_name="test-app",
        build_id="test-build-id",
        project_name="test-project",
        architectures=["amd64"],
        project_dir=Path(),
        timeout=0,
    )


@pytest.fixture()
def mock_git_check(mocker):
    """Ignore git check."""
    mock_git_check = mocker.patch(
        "snapcraft.remote.git.check_git_repo_for_remote_build"
    )
    mock_git_check.return_value = None
    return mock_git_check


@pytest.fixture()
def mock_git_type_check(mocker):
    """Ignore git type check."""
    mock_git_type = mocker.patch("snapcraft.remote.git.get_git_repo_type")
    mock_git_type.return_value = GitType.NORMAL
    return mock_git_type


def test_remote_builder_init(mock_launchpad_client, mock_worktree):
    """Verify remote builder is properly initialized."""
    RemoteBuilder(
        app_name="test-app",
        build_id="test-build-id",
        project_name="test-project",
        architectures=["amd64"],
        project_dir=Path(),
        timeout=10,
    )

    assert mock_launchpad_client.mock_calls == [
        call(
            app_name="test-app",
            build_id="test-build-id",
            project_name="test-project",
            architectures=["amd64"],
            timeout=10,
        )
    ]
    assert mock_worktree.mock_calls == [
        call(app_name="test-app", build_id="test-build-id", project_dir=Path())
    ]


@pytest.mark.usefixtures("mock_git_check", "mock_git_type_check")
@pytest.mark.usefixtures("new_dir", "mock_git_check")
def test_build_id_computed():
    """Compute a build id if it is not provided."""
    remote_builder = RemoteBuilder(
        app_name="test-app",
        build_id=None,
        project_name="test-project",
        architectures=["amd64"],
        project_dir=Path(),
        timeout=0,
    )

    assert re.match("test-app-test-project-[0-9a-f]{32}", remote_builder.build_id)


@pytest.mark.parametrize("archs", (["amd64"], _SUPPORTED_ARCHS))
def test_validate_architectures_supported(archs):
    """Supported architectures should not raise an error."""
    RemoteBuilder(
        app_name="test-app",
        build_id="test-build-id",
        project_name="test-project",
        architectures=archs,
        project_dir=Path(),
        timeout=0,
    )


@pytest.mark.parametrize(
    "archs",
    [
        # unsupported
        ["bad"],
        # supported and unsupported
        ["amd64", "bad"],
        # multiple supported and unsupported
        ["bad", "amd64", "bad2", "arm64"],
    ],
)
def test_validate_architectures_unsupported(archs):
    """Raise an error for unsupported architectures."""
    with pytest.raises(UnsupportedArchitectureError):
        RemoteBuilder(
            app_name="test-app",
            build_id="test-build-id",
            project_name="test-project",
            architectures=archs,
            project_dir=Path(),
            timeout=0,
        )


@pytest.mark.usefixtures("mock_git_check", "mock_git_type_check")
@patch("logging.Logger.info")
def test_print_status_builds_found(
    mock_log, mock_launchpad_client, fake_remote_builder
):
    """Print the status of a remote build."""
    mock_launchpad_client.return_value.has_outstanding_build.return_value = True
    mock_launchpad_client.return_value.get_build_status.return_value = {
        "amd64": "Needs building",
        "arm64": "Currently building",
    }

    fake_remote_builder.print_status()

    assert mock_log.mock_calls == [
        call("Build status for arch %s: %s", "amd64", "Needs building"),
        call("Build status for arch %s: %s", "arm64", "Currently building"),
    ]


@pytest.mark.usefixtures("mock_git_check", "mock_git_type_check")
@patch("logging.Logger.info")
def test_print_status_no_build_found(mock_log, fake_remote_builder):
    """Print the status of a remote build."""
    fake_remote_builder.print_status()

    assert mock_log.mock_calls == [call("No build task(s) found.")]


@pytest.mark.usefixtures("mock_git_check", "mock_git_type_check")
@pytest.mark.parametrize("has_builds", (True, False))
def test_has_outstanding_build(has_builds, fake_remote_builder, mock_launchpad_client):
    """Check for outstanding builds."""
    mock_launchpad_client.return_value.has_outstanding_build.return_value = has_builds

    assert fake_remote_builder.has_outstanding_build() == has_builds


@pytest.mark.usefixtures("mock_git_check", "mock_git_type_check")
def test_monitor_build(fake_remote_builder, mock_launchpad_client):
    """Monitor a build."""
    fake_remote_builder.monitor_build()

    mock_launchpad_client.return_value.monitor_build.assert_called_once()


@pytest.mark.usefixtures("mock_git_check", "mock_git_type_check")
def test_clean_build(fake_remote_builder, mock_launchpad_client, mock_worktree):
    """Clean a build."""
    fake_remote_builder.clean_build()

    mock_launchpad_client.return_value.cleanup.assert_called_once()
    mock_worktree.return_value.clean_cache.assert_called_once()


@pytest.mark.usefixtures("mock_git_check", "mock_git_type_check")
def test_start_build(fake_remote_builder, mock_launchpad_client, mock_worktree):
    """Start a build."""
    fake_remote_builder.start_build()

    mock_worktree.return_value.init_repo.assert_called_once()
    mock_launchpad_client.return_value.push_source_tree.assert_called_once()
    mock_launchpad_client.return_value.start_build.assert_called_once()
