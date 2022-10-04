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

import re
from unittest.mock import call

import pytest
from craft_providers import ProviderError, bases
from craft_providers.lxd import LXDError, LXDInstallationError

from snapcraft import providers


@pytest.fixture
def mock_buildd_base_configuration(mocker):
    mock_base_config = mocker.patch(
        "snapcraft.providers._lxd.bases.BuilddBase", autospec=True
    )
    mock_base_config.compatibility_tag = "buildd-base-v0"
    yield mock_base_config


@pytest.fixture
def mock_configure_buildd_image_remote(mocker):
    yield mocker.patch(
        "craft_providers.lxd.configure_buildd_image_remote",
        return_value="buildd-remote",
    )


@pytest.fixture
def mock_lxc(mocker):
    yield mocker.patch("craft_providers.lxd.LXC", autospec=True)


@pytest.fixture(autouse=True)
def mock_lxd_ensure_lxd_is_ready(mocker):
    yield mocker.patch("craft_providers.lxd.ensure_lxd_is_ready", return_value=None)


@pytest.fixture
def mock_lxd_install(mocker):
    yield mocker.patch("craft_providers.lxd.install")


@pytest.fixture(autouse=True)
def mock_lxd_is_installed(mocker):
    yield mocker.patch("craft_providers.lxd.is_installed", return_value=True)


@pytest.fixture
def mock_lxd_launch(mocker):
    yield mocker.patch("craft_providers.lxd.launch", autospec=True)


@pytest.fixture
def mock_confirm_with_user(mocker):
    yield mocker.patch(
        "snapcraft.providers._lxd.utils.confirm_with_user", return_value=True
    )


def test_ensure_provider_is_available_ok_when_installed(mock_lxd_is_installed):
    mock_lxd_is_installed.return_value = True
    provider = providers.LXDProvider()

    provider.ensure_provider_is_available()


def test_ensure_provider_is_available_errors_when_user_says_no(
    mock_lxd_is_installed, mock_lxd_install, mock_confirm_with_user
):
    mock_confirm_with_user.return_value = False
    mock_lxd_is_installed.return_value = False
    provider = providers.LXDProvider()

    with pytest.raises(
        ProviderError,
        match=re.escape(
            "LXD is required, but not installed. Visit https://snapcraft.io/lxd for "
            "instructions on how to install the LXD snap for your distribution"
        ),
    ):
        provider.ensure_provider_is_available()


def test_ensure_provider_is_available_errors_when_lxd_install_fails(
    mock_lxd_is_installed, mock_lxd_install, mock_confirm_with_user
):
    error = LXDInstallationError("foo")
    mock_lxd_is_installed.return_value = False
    mock_lxd_install.side_effect = error
    provider = providers.LXDProvider()

    with pytest.raises(
        ProviderError,
        match=re.escape(
            "Failed to install LXD. Visit https://snapcraft.io/lxd for "
            "instructions on how to install the LXD snap for your distribution"
        ),
    ) as raised:
        provider.ensure_provider_is_available()

    assert raised.value.__cause__ is error


def test_ensure_provider_is_available_errors_when_lxd_not_ready(
    mock_lxd_is_installed,
    mock_lxd_install,
    mock_lxd_ensure_lxd_is_ready,
):
    error = LXDError(
        brief="some error", details="some details", resolution="some resolution"
    )
    mock_lxd_is_installed.return_value = True
    mock_lxd_ensure_lxd_is_ready.side_effect = error
    provider = providers.LXDProvider()

    with pytest.raises(
        ProviderError,
        match=re.escape("some error\nsome details\nsome resolution"),
    ) as raised:
        provider.ensure_provider_is_available()

    assert raised.value.__cause__ is error


@pytest.mark.parametrize("is_installed", [True, False])
def test_is_provider_installed(is_installed, mock_lxd_is_installed):
    mock_lxd_is_installed.return_value = is_installed
    provider = providers.LXDProvider()

    assert provider.is_provider_installed() == is_installed


def test_create_environment(mocker):
    mock_lxd_instance = mocker.patch("snapcraft.providers._lxd.lxd.LXDInstance")

    provider = providers.LXDProvider()
    provider.create_environment(instance_name="test-name")

    mock_lxd_instance.assert_called_once_with(
        name="test-name",
        project="snapcraft",
        remote="local",
    )


def test_launched_environment(
    mock_buildd_base_configuration,
    mock_configure_buildd_image_remote,
    mock_lxd_launch,
    tmp_path,
):
    provider = providers.LXDProvider()

    with provider.launched_environment(
        project_name="test-project",
        project_path=tmp_path,
        base_configuration=mock_buildd_base_configuration,
        build_base=bases.BuilddBaseAlias.JAMMY.value,
        instance_name="test-instance-name",
    ) as instance:
        assert instance is not None
        assert mock_configure_buildd_image_remote.mock_calls == [call()]
        assert mock_lxd_launch.mock_calls == [
            call(
                name="test-instance-name",
                base_configuration=mock_buildd_base_configuration,
                image_name="core22",
                image_remote="buildd-remote",
                auto_clean=True,
                auto_create_project=True,
                map_user_uid=True,
                uid=tmp_path.stat().st_uid,
                use_snapshots=True,
                project="snapcraft",
                remote="local",
            ),
        ]

        mock_lxd_launch.reset_mock()

    assert mock_lxd_launch.mock_calls == [
        call().unmount_all(),
        call().stop(),
    ]


def test_launched_environment_launch_base_configuration_error(
    mock_buildd_base_configuration,
    mock_configure_buildd_image_remote,
    mock_lxd_launch,
    tmp_path,
):
    error = bases.BaseConfigurationError(brief="fail")
    mock_lxd_launch.side_effect = error
    provider = providers.LXDProvider()

    with pytest.raises(ProviderError, match="fail") as raised:
        with provider.launched_environment(
            project_name="test-project",
            project_path=tmp_path,
            base_configuration=mock_buildd_base_configuration,
            build_base=bases.BuilddBaseAlias.FOCAL.value,
            instance_name="test-instance-name",
        ):
            pass

    assert raised.value.__cause__ is error


def test_launched_environment_launch_lxd_error(
    mock_buildd_base_configuration,
    mock_configure_buildd_image_remote,
    mock_lxd_launch,
    tmp_path,
):
    error = LXDError(brief="fail")
    mock_lxd_launch.side_effect = error
    provider = providers.LXDProvider()

    with pytest.raises(ProviderError, match="fail") as raised:
        with provider.launched_environment(
            project_name="test-project",
            project_path=tmp_path,
            base_configuration=mock_buildd_base_configuration,
            build_base=bases.BuilddBaseAlias.FOCAL.value,
            instance_name="test-instance-name",
        ):
            pass

    assert raised.value.__cause__ is error


def test_launched_environment_unmounts_and_stops_after_error(
    mock_buildd_base_configuration,
    mock_configure_buildd_image_remote,
    mock_lxd_launch,
    tmp_path,
):
    provider = providers.LXDProvider()

    with pytest.raises(RuntimeError):
        with provider.launched_environment(
            project_name="test-project",
            project_path=tmp_path,
            base_configuration=mock_buildd_base_configuration,
            build_base=bases.BuilddBaseAlias.FOCAL.value,
            instance_name="test-instance-name",
        ):
            mock_lxd_launch.reset_mock()
            raise RuntimeError("this is a test")

    assert mock_lxd_launch.mock_calls == [
        call().unmount_all(),
        call().stop(),
    ]


def test_launched_environment_unmount_all_error(
    mock_buildd_base_configuration,
    mock_configure_buildd_image_remote,
    mock_lxd_launch,
    tmp_path,
):
    error = LXDError(brief="fail")
    mock_lxd_launch.return_value.unmount_all.side_effect = error
    provider = providers.LXDProvider()

    with pytest.raises(ProviderError, match="fail") as raised:
        with provider.launched_environment(
            project_name="test-project",
            project_path=tmp_path,
            base_configuration=mock_buildd_base_configuration,
            build_base=bases.BuilddBaseAlias.FOCAL.value,
            instance_name="test-instance-name",
        ):
            pass

    assert raised.value.__cause__ is error


def test_launched_environment_stop_error(
    mock_buildd_base_configuration,
    mock_configure_buildd_image_remote,
    mock_lxd_launch,
    tmp_path,
):
    error = LXDError(brief="fail")
    mock_lxd_launch.return_value.stop.side_effect = error
    provider = providers.LXDProvider()

    with pytest.raises(ProviderError, match="fail") as raised:
        with provider.launched_environment(
            project_name="test-project",
            project_path=tmp_path,
            base_configuration=mock_buildd_base_configuration,
            build_base=bases.BuilddBaseAlias.JAMMY.value,
            instance_name="test-instance-name",
        ):
            pass

    assert raised.value.__cause__ is error
