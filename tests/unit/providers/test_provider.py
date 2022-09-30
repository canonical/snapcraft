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


@pytest.fixture()
def mock_default_command_environment():
    with patch(
        "craft_providers.bases.buildd.default_command_environment",
        return_value=dict(PATH="test-path"),
    ) as mock_environment:
        yield mock_environment


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


def test_get_command_environment(mocker, mock_default_command_environment):
    """Verify command environment is properly constructed."""
    provider = providers.LXDProvider()
    command_environment = provider.get_command_environment()

    assert command_environment == {"PATH": "test-path", "SNAPCRAFT_MANAGED_MODE": "1"}


def test_get_command_environment_http_https_proxy(
    mocker, mock_default_command_environment
):
    """Verify http and https proxies are added to the environment."""
    provider = providers.LXDProvider()
    command_environment = provider.get_command_environment(
        http_proxy="test-http", https_proxy="test-https"
    )

    assert command_environment == {
        "PATH": "test-path",
        "SNAPCRAFT_MANAGED_MODE": "1",
        "http_proxy": "test-http",
        "https_proxy": "test-https",
    }


def test_get_command_environment_passthrough(
    mocker, mock_default_command_environment, monkeypatch
):
    """Verify variables from the environment are passed to the command environment."""
    monkeypatch.setenv("http_proxy", "test-http")
    monkeypatch.setenv("https_proxy", "test-https")
    monkeypatch.setenv("no_proxy", "test-no-proxy")
    monkeypatch.setenv("SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS", "test-extensions")
    monkeypatch.setenv("SNAPCRAFT_BUILD_FOR", "test-build-for")
    monkeypatch.setenv("SNAPCRAFT_BUILD_INFO", "test-build-info")
    monkeypatch.setenv("SNAPCRAFT_IMAGE_INFO", "test-image-info")

    # ensure other variables are not being passed
    monkeypatch.setenv("other_var", "test-other-var")

    provider = providers.LXDProvider()
    command_environment = provider.get_command_environment()

    assert command_environment == {
        "PATH": "test-path",
        "SNAPCRAFT_MANAGED_MODE": "1",
        "http_proxy": "test-http",
        "https_proxy": "test-https",
        "no_proxy": "test-no-proxy",
        "SNAPCRAFT_ENABLE_EXPERIMENTAL_EXTENSIONS": "test-extensions",
        "SNAPCRAFT_BUILD_FOR": "test-build-for",
        "SNAPCRAFT_BUILD_INFO": "test-build-info",
        "SNAPCRAFT_IMAGE_INFO": "test-image-info",
    }


def test_get_command_environment_http_https_priority(
    mocker, mock_default_command_environment, monkeypatch
):
    """Verify http and https proxies from the function argument take priority over the
    proxies defined in the environment."""
    monkeypatch.setenv("http_proxy", "test-http-from-env")
    monkeypatch.setenv("https_proxy", "test-https-from-env")

    provider = providers.LXDProvider()
    command_environment = provider.get_command_environment(
        http_proxy="test-http-from-arg", https_proxy="test-https-from-arg"
    )

    assert command_environment == {
        "PATH": "test-path",
        "SNAPCRAFT_MANAGED_MODE": "1",
        "http_proxy": "test-http-from-arg",
        "https_proxy": "test-https-from-arg",
    }
