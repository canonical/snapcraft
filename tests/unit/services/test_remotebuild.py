# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2024 Canonical Ltd.
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

"""Tests for the Snapcraft Remote Build service."""
from unittest import mock

import pytest


def test_remotebuild_fetch_logs(mocker, tmp_path, remote_build_service):
    """Test that logs are fetched"""
    remote_build_service._is_setup = True
    remote_build_service._name = "snapcraft-mytest-7f417e8455caa67e89ba6a1f3f6bed18"

    remote_build_service._builds = [
        mock.Mock(build_log_url="http://whatever", arch_tag="riscv64")
    ]
    remote_build_service.request.download_files_with_progress = mock.Mock()
    logs = remote_build_service.fetch_logs(tmp_path)

    remote_build_service.request.download_files_with_progress.assert_called_once()

    assert "riscv64" in logs


def test_remotebuild_fetch_no_setup(mocker, tmp_path, remote_build_service):
    """Test that fetch operations raise a RuntimeError if the service is not set up."""
    remote_build_service._is_setup = False

    with pytest.raises(RuntimeError):
        all(remote_build_service.monitor_builds())
    with pytest.raises(RuntimeError):
        remote_build_service.fetch_logs(tmp_path)
    with pytest.raises(RuntimeError):
        remote_build_service.fetch_artifacts(tmp_path)


def test_new_snap_recipe(mocker, remote_build_service):
    """Test that a new SnapRecipe is created."""
    mock_new_recipe = mocker.patch("craft_application.launchpad.models.SnapRecipe.new")
    git_repo = mock.Mock(self_link="http://whatever")

    remote_build_service._lp_project = mock.Mock()
    remote_build_service._lp_project.name = "mytest"

    remote_build_service._new_recipe(
        name="mytest",
        repository=git_repo,
        architectures=["riscv64"],
    )

    mock_new_recipe.assert_called_once_with(
        remote_build_service.lp,
        "mytest",
        "craft_test_user",
        architectures=["riscv64"],
        project="mytest",
        git_ref=git_repo.self_link + "/+ref/main",
    )
