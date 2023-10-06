# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2023 Canonical Ltd
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

"""Unit tests for the worktree module."""

from pathlib import Path
from unittest.mock import call

import pytest

from snapcraft.remote import WorkTree


@pytest.fixture(autouse=True)
def mock_git_repo(mocker):
    """Returns a mocked GitRepo."""
    return mocker.patch("snapcraft.remote.worktree.GitRepo")


@pytest.fixture(autouse=True)
def mock_base_directory(mocker, new_dir):
    """Returns a mocked `xdg.BaseDirectory`."""
    _mock_base_directory = mocker.patch("snapcraft.remote.worktree.BaseDirectory")
    _mock_base_directory.save_cache_path.return_value = new_dir
    return _mock_base_directory


@pytest.fixture(autouse=True)
def mock_copytree(mocker):
    """Returns a mocked `shutil.copytree()`."""
    return mocker.patch("snapcraft.remote.worktree.copytree")


def test_worktree_init_clean(
    mock_base_directory, mock_copytree, mock_git_repo, new_dir
):
    """Test initialization of a WorkTree with a clean git repository."""
    mock_git_repo.return_value.is_clean.return_value = True

    worktree = WorkTree(app_name="test-app", build_id="test-id", project_dir=Path())
    worktree.init_repo()

    assert isinstance(worktree, WorkTree)
    mock_base_directory.save_cache_path.assert_called_with(
        "test-app", "remote-build", "test-id"
    )
    assert mock_git_repo.mock_calls == [
        call(Path().resolve() / "repo"),
        call().is_clean(),
    ]


def test_worktree_init_dirty(
    mock_base_directory, mock_copytree, mock_git_repo, new_dir
):
    """Test initialization of a WorkTree with a clean git repository."""
    mock_git_repo.return_value.is_clean.return_value = False

    worktree = WorkTree(app_name="test-app", build_id="test-id", project_dir=Path())
    worktree.init_repo()

    assert isinstance(worktree, WorkTree)
    mock_base_directory.save_cache_path.assert_called_with(
        "test-app", "remote-build", "test-id"
    )
    assert mock_git_repo.mock_calls == [
        call(Path().resolve() / "repo"),
        call().is_clean(),
        call().add_all(),
        call().commit(),
    ]


def test_worktree_repo_dir(new_dir):
    """Verify the `repo_dir` property."""
    worktree = WorkTree(app_name="test-app", build_id="test-id", project_dir=Path())

    assert worktree.repo_dir == Path().resolve() / "repo"
