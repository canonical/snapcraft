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
        git_ref=git_repo.git_https_url,
    )
