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

from unittest.mock import call, patch

import pytest
from craft_providers import ProviderError
from craft_providers.lxd import LXDError

from snapcraft import providers


@pytest.fixture
def lxd_instance_mock(mocker):
    instance_mock = mocker.patch(
        "craft_providers.lxd.launcher.LXDInstance", autospec=True
    )
    instance_mock.return_value.name = "test-instance"
    instance_mock.return_value.project = "test-project"
    instance_mock.return_value.remote = "test-remote"
    yield instance_mock


@pytest.fixture()
def mock_lxd_is_installed():
    with patch(
        "craft_providers.lxd.is_installed", return_value=True
    ) as mock_is_installed:
        yield mock_is_installed


@pytest.fixture()
def mock_lxd_delete():
    with patch("craft_providers.lxd.LXDInstance.delete") as mock_delete:
        yield mock_delete


@pytest.fixture()
def mock_lxd_exists():
    with patch(
        "craft_providers.lxd.LXDInstance.exists", return_value=True
    ) as mock_exists:
        yield mock_exists


def test_clean_project_environment_exists(
    mock_lxd_exists,
    mock_lxd_delete,
    mock_lxd_is_installed,
    new_dir,
):
    """Assert instance is deleted if it exists."""
    provider = providers.LXDProvider()
    provider.clean_project_environments(
        project_name="test",
        project_path=new_dir,
        build_on="test",
        build_for="test",
    )

    assert mock_lxd_delete.mock_calls == [call()]


def test_clean_project_environment_does_not_exist(
    mock_lxd_exists,
    mock_lxd_delete,
    mock_lxd_is_installed,
    new_dir,
):
    """Assert instance is not deleted if it does not exist."""
    mock_lxd_exists.return_value = False

    provider = providers.LXDProvider()
    provider.clean_project_environments(
        project_name="test",
        project_path=new_dir,
        build_on="test",
        build_for="test",
    )

    assert mock_lxd_delete.mock_calls == []


def test_clean_project_environment_lxd_not_installed(
    mock_lxd_delete,
    mock_lxd_is_installed,
    new_dir,
):
    """Assert instance is not deleted if LXD is not installed."""
    mock_lxd_is_installed.return_value = False

    provider = providers.LXDProvider()
    provider.clean_project_environments(
        project_name="test",
        project_path=new_dir,
        build_on="test",
        build_for="test",
    )

    assert mock_lxd_delete.mock_calls == []


def test_clean_project_environment_exists_error(
    mock_lxd_exists, mock_lxd_delete, mock_lxd_is_installed, new_dir
):
    """Assert error on `exists` call is caught."""
    error = LXDError("fail")
    mock_lxd_exists.side_effect = error
    provider = providers.LXDProvider()

    with pytest.raises(ProviderError) as raised:
        provider.clean_project_environments(
            project_name="test",
            project_path=new_dir,
            build_on="test",
            build_for="test",
        )

    assert str(raised.value) == "fail"


def test_clean_project_environment_delete_error(
    mock_lxd_exists, mock_lxd_delete, mock_lxd_is_installed, new_dir
):
    """Assert error on `delete` call is caught."""
    error = LXDError("fail")
    mock_lxd_delete.side_effect = error
    provider = providers.LXDProvider()

    with pytest.raises(ProviderError) as raised:
        provider.clean_project_environments(
            project_name="test",
            project_path=new_dir,
            build_on="test",
            build_for="test",
        )

    assert str(raised.value) == "fail"
