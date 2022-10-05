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
from craft_providers.multipass import MultipassError, MultipassInstallationError

from snapcraft import providers


@pytest.fixture
def mock_buildd_base_configuration(mocker):
    mock_base_config = mocker.patch(
        "snapcraft.providers._multipass.bases.BuilddBase", autospec=True
    )
    mock_base_config.compatibility_tag = "buildd-base-v0"
    yield mock_base_config


@pytest.fixture
def mock_multipass(mocker):
    yield mocker.patch("craft_providers.multipass.Multipass", autospec=True)


@pytest.fixture(autouse=True)
def mock_multipass_ensure_multipass_is_ready(mocker):
    yield mocker.patch(
        "craft_providers.multipass.ensure_multipass_is_ready", return_value=None
    )


@pytest.fixture
def mock_multipass_install(mocker):
    yield mocker.patch("craft_providers.multipass.install")


@pytest.fixture(autouse=True)
def mock_multipass_is_installed(mocker):
    yield mocker.patch("craft_providers.multipass.is_installed", return_value=True)


@pytest.fixture
def mock_multipass_launch(mocker):
    yield mocker.patch("craft_providers.multipass.launch", autospec=True)


@pytest.fixture
def mock_confirm_with_user(mocker):
    yield mocker.patch(
        "snapcraft.providers._multipass.utils.confirm_with_user", return_value=True
    )


def test_ensure_provider_is_available_ok_when_installed(mock_multipass_is_installed):
    mock_multipass_is_installed.return_value = True
    provider = providers.MultipassProvider()

    provider.ensure_provider_is_available()


def test_ensure_provider_is_available_errors_when_user_says_no(
    mock_multipass_is_installed, mock_multipass_install, mock_confirm_with_user
):
    mock_confirm_with_user.return_value = False
    mock_multipass_is_installed.return_value = False
    provider = providers.MultipassProvider()

    with pytest.raises(
        ProviderError,
        match=re.escape(
            "Multipass is required, but not installed. Visit https://multipass.run/ "
            "for instructions on installing Multipass for your operating system."
        ),
    ):
        provider.ensure_provider_is_available()


def test_ensure_provider_is_available_errors_when_multipass_install_fails(
    mock_multipass_is_installed, mock_multipass_install, mock_confirm_with_user
):
    error = MultipassInstallationError("foo")
    mock_multipass_is_installed.return_value = False
    mock_multipass_install.side_effect = error
    provider = providers.MultipassProvider()

    match = re.escape(
        "Failed to install Multipass. Visit https://multipass.run/ for "
        "instructions on installing Multipass for your operating system."
    )
    with pytest.raises(ProviderError, match=match) as raised:
        provider.ensure_provider_is_available()

    assert raised.value.__cause__ is error


def test_ensure_provider_is_available_errors_when_multipass_not_ready(
    mock_multipass_is_installed,
    mock_multipass_install,
    mock_multipass_ensure_multipass_is_ready,
    mock_confirm_with_user,
):
    error = MultipassError(
        brief="some error", details="some details", resolution="some resolution"
    )
    mock_multipass_is_installed.return_value = True
    mock_multipass_ensure_multipass_is_ready.side_effect = error
    provider = providers.MultipassProvider()

    with pytest.raises(
        ProviderError,
        match=re.escape("some error\nsome details\nsome resolution"),
    ) as raised:
        provider.ensure_provider_is_available()

    assert raised.value.__cause__ is error


@pytest.mark.parametrize("is_installed", [True, False])
def test_is_provider_installed(is_installed, mock_multipass_is_installed):
    mock_multipass_is_installed.return_value = is_installed
    provider = providers.MultipassProvider()

    assert provider.is_provider_installed() == is_installed


def test_create_environment(mocker):
    mock_multipass_instance = mocker.patch(
        "snapcraft.providers._multipass.multipass.MultipassInstance"
    )

    provider = providers.MultipassProvider()
    provider.create_environment(instance_name="test-name")

    mock_multipass_instance.assert_called_once_with(name="test-name")


def test_launched_environment(
    mock_buildd_base_configuration,
    mock_multipass_launch,
    tmp_path,
):
    provider = providers.MultipassProvider()

    with provider.launched_environment(
        project_name="test-project",
        project_path=tmp_path,
        base_configuration=mock_buildd_base_configuration,
        build_base=bases.BuilddBaseAlias.JAMMY.value,
        instance_name="test-instance-name",
    ) as instance:
        assert instance is not None
        assert mock_multipass_launch.mock_calls == [
            call(
                name="test-instance-name",
                base_configuration=mock_buildd_base_configuration,
                image_name="snapcraft:22.04",
                cpus=2,
                disk_gb=64,
                mem_gb=2,
                auto_clean=True,
            ),
        ]
        mock_multipass_launch.reset_mock()

    assert mock_multipass_launch.mock_calls == [
        call().unmount_all(),
        call().stop(),
    ]


def test_launched_environment_unmounts_and_stops_after_error(
    mock_buildd_base_configuration, mock_multipass_launch, tmp_path
):
    provider = providers.MultipassProvider()

    with pytest.raises(RuntimeError):
        with provider.launched_environment(
            project_name="test-project",
            project_path=tmp_path,
            base_configuration=mock_buildd_base_configuration,
            build_base=bases.BuilddBaseAlias.FOCAL.value,
            instance_name="test-instance-name",
        ):
            mock_multipass_launch.reset_mock()
            raise RuntimeError("this is a test")

    assert mock_multipass_launch.mock_calls == [
        call().unmount_all(),
        call().stop(),
    ]


def test_launched_environment_launch_base_configuration_error(
    mock_buildd_base_configuration, mock_multipass_launch, tmp_path
):
    error = bases.BaseConfigurationError(brief="fail")
    mock_multipass_launch.side_effect = error
    provider = providers.MultipassProvider()

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


def test_launched_environment_launch_multipass_error(
    mock_buildd_base_configuration, mock_multipass_launch, tmp_path
):
    error = MultipassError(brief="fail")
    mock_multipass_launch.side_effect = error
    provider = providers.MultipassProvider()

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


def test_launched_environment_unmount_all_error(
    mock_buildd_base_configuration, mock_multipass_launch, tmp_path
):
    error = MultipassError(brief="fail")
    mock_multipass_launch.return_value.unmount_all.side_effect = error
    provider = providers.MultipassProvider()

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
    mock_buildd_base_configuration, mock_multipass_launch, tmp_path
):
    error = MultipassError(brief="fail")
    mock_multipass_launch.return_value.stop.side_effect = error
    provider = providers.MultipassProvider()

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
