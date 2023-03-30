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

from pathlib import Path
from unittest.mock import ANY, MagicMock, Mock, call, patch

import pytest
from craft_providers import ProviderError, bases
from craft_providers.lxd import LXDProvider
from craft_providers.multipass import MultipassProvider

from snapcraft import providers
from snapcraft.snap_config import SnapConfig


@pytest.fixture()
def mock_default_command_environment():
    with patch(
        "craft_providers.bases.buildd.default_command_environment",
        return_value=dict(PATH="test-path"),
    ) as mock_environment:
        yield mock_environment


def test_capture_logs_from_instance(mocker, emitter, mock_instance, new_dir):
    """Verify logs from an instance are retrieved and emitted."""
    fake_log = Path(new_dir / "fake.file")
    fake_log_data = "some\nlog data\nhere"
    fake_log.write_text(fake_log_data, encoding="utf-8")

    mock_instance.temporarily_pull_file = MagicMock()
    mock_instance.temporarily_pull_file.return_value = fake_log

    providers.capture_logs_from_instance(mock_instance)

    assert mock_instance.mock_calls == [
        call.temporarily_pull_file(source=Path("/tmp/snapcraft.log"), missing_ok=True)
    ]
    expected = [
        call("debug", "Logs retrieved from managed instance:"),
        call("debug", ":: some"),
        call("debug", ":: log data"),
        call("debug", ":: here"),
    ]
    emitter.assert_interactions(expected)


def test_capture_log_from_instance_not_found(mocker, emitter, mock_instance, new_dir):
    """Verify a missing log file is handled properly."""
    mock_instance.temporarily_pull_file = MagicMock(return_value=None)
    mock_instance.temporarily_pull_file.return_value = (
        mock_instance.temporarily_pull_file
    )
    mock_instance.temporarily_pull_file.__enter__ = Mock(return_value=None)

    providers.capture_logs_from_instance(mock_instance)

    emitter.assert_debug("Could not find log file /tmp/snapcraft.log in instance.")
    mock_instance.temporarily_pull_file.assert_called_with(
        source=Path("/tmp/snapcraft.log"), missing_ok=True
    )


@pytest.mark.parametrize(
    "is_provider_installed, confirm_with_user",
    [(True, True), (True, False), (False, True)],
)
def test_ensure_provider_is_available_lxd(
    is_provider_installed, confirm_with_user, mocker
):
    """Verify LXD is ensured to be available when LXD is installed or the user chooses
    to install LXD."""
    mock_lxd_provider = Mock(spec=LXDProvider)
    mocker.patch(
        "snapcraft.providers.LXDProvider.is_provider_installed",
        return_value=is_provider_installed,
    )
    mocker.patch(
        "snapcraft.providers.confirm_with_user",
        return_value=confirm_with_user,
    )
    mock_ensure_provider_is_available = mocker.patch(
        "snapcraft.providers.ensure_provider_is_available"
    )

    providers.ensure_provider_is_available(mock_lxd_provider)

    mock_ensure_provider_is_available.assert_called_once()


def test_ensure_provider_is_available_lxd_error(mocker):
    """Raise an error if the user does not choose to install LXD."""
    mock_lxd_provider = Mock(spec=LXDProvider)
    mocker.patch(
        "snapcraft.providers.LXDProvider.is_provider_installed",
        return_value=False,
    )
    mocker.patch("snapcraft.providers.confirm_with_user", return_value=False)

    with pytest.raises(ProviderError) as error:
        providers.ensure_provider_is_available(mock_lxd_provider)

    assert error.value.brief == (
        "LXD is required, but not installed. Visit https://snapcraft.io/lxd for "
        "instructions on how to install the LXD snap for your distribution"
    )


@pytest.mark.parametrize(
    "is_provider_installed, confirm_with_user",
    [(True, True), (True, False), (False, True)],
)
def test_ensure_provider_is_available_multipass(
    is_provider_installed, confirm_with_user, mocker
):
    """Verify Multipass is ensured to be available when Multipass is installed or the
    user chooses to install Multipass."""
    mock_multipass_provider = Mock(spec=MultipassProvider)
    mocker.patch(
        "snapcraft.providers.MultipassProvider.is_provider_installed",
        return_value=is_provider_installed,
    )
    mocker.patch(
        "snapcraft.providers.confirm_with_user",
        return_value=confirm_with_user,
    )
    mock_ensure_provider_is_available = mocker.patch(
        "snapcraft.providers.ensure_provider_is_available"
    )

    providers.ensure_provider_is_available(mock_multipass_provider)

    mock_ensure_provider_is_available.assert_called_once()


def test_ensure_provider_is_available_multipass_error(mocker):
    """Raise an error if the user does not choose to install Multipass."""
    mock_multipass_provider = Mock(spec=MultipassProvider)
    mocker.patch(
        "snapcraft.providers.MultipassProvider.is_provider_installed",
        return_value=False,
    )
    mocker.patch("snapcraft.providers.confirm_with_user", return_value=False)

    with pytest.raises(ProviderError) as error:
        providers.ensure_provider_is_available(mock_multipass_provider)

    assert error.value.brief == (
        "Multipass is required, but not installed. Visit https://multipass.run/for "
        "instructions on installing Multipass for your operating system."
    )


def test_ensure_provider_is_available_unknown_error():
    """Raise an error if the provider type is unknown."""
    mock_multipass_provider = Mock()

    with pytest.raises(ProviderError) as error:
        providers.ensure_provider_is_available(mock_multipass_provider)

    assert error.value.brief == "cannot install unknown provider"


@pytest.mark.parametrize("alias", providers.SNAPCRAFT_BASE_TO_PROVIDER_BASE.values())
def test_get_base_configuration(
    alias,
    tmp_path,
    mocker,
    monkeypatch,
):
    """Verify the base configuration is properly configured."""
    mocker.patch("sys.platform", "linux")
    mocker.patch(
        "snapcraft.providers.get_managed_environment_snap_channel",
        return_value="test-channel",
    )
    mocker.patch(
        "snapcraft.providers.get_command_environment",
        return_value="test-env",
    )
    mocker.patch(
        "snapcraft.providers.get_instance_name",
        return_value="test-instance-name",
    )
    mock_buildd_base = mocker.patch("snapcraft.providers.bases.BuilddBase")
    mock_buildd_base.compatibility_tag = "buildd-base-v0"
    monkeypatch.setenv("SNAP_INSTANCE_NAME", "test-snap-name")

    providers.get_base_configuration(
        alias=alias,
        instance_name="test-instance-name",
    )

    mock_buildd_base.assert_called_with(
        alias=alias,
        compatibility_tag="snapcraft-buildd-base-v0.0",
        environment="test-env",
        hostname="test-instance-name",
        snaps=[
            bases.buildd.Snap(
                name="test-snap-name", channel="test-channel", classic=True
            )
        ],
        packages=["gnupg", "dirmngr", "git"],
    )


@pytest.mark.parametrize(
    ["platform", "snap_channel"],
    [
        # default to snapcraft from the stable channel on non-linux systems
        ("darwin", "stable"),
        ("win32", "stable"),
        # Linux hosts support snap injection, so there should be no default channel
        ("linux", None),
    ],
)
def test_get_base_configuration_snap_channel(
    platform,
    snap_channel,
    tmp_path,
    mocker,
    monkeypatch,
):
    """Verify the correct snap channel is chosen for different host OS's."""
    mocker.patch("sys.platform", platform)
    mocker.patch(
        "snapcraft.providers.get_managed_environment_snap_channel",
        return_value=None,
    )
    mocker.patch("snapcraft.providers.get_command_environment")
    mocker.patch("snapcraft.providers.get_instance_name")
    mock_buildd_base = mocker.patch("snapcraft.providers.bases.BuilddBase")
    monkeypatch.setenv("SNAP_INSTANCE_NAME", "snapcraft")

    providers.get_base_configuration(
        alias=bases.BuilddBaseAlias.JAMMY,
        instance_name="test-instance-name",
    )

    mock_buildd_base.assert_called_with(
        alias=ANY,
        compatibility_tag=ANY,
        environment=ANY,
        hostname=ANY,
        snaps=[bases.buildd.Snap(name="snapcraft", channel=snap_channel, classic=True)],
        packages=ANY,
    )


def test_get_base_configuration_snap_instance_name_default(
    tmp_path,
    mocker,
    monkeypatch,
):
    """If `SNAP_INSTANCE_NAME` does not exist, use the default name 'snapcraft'."""
    mocker.patch("sys.platform", "linux")
    mocker.patch(
        "snapcraft.providers.get_managed_environment_snap_channel",
        return_value=None,
    )
    mocker.patch("snapcraft.providers.get_command_environment")
    mocker.patch("snapcraft.providers.get_instance_name")
    mock_buildd_base = mocker.patch("snapcraft.providers.bases.BuilddBase")
    monkeypatch.delenv("SNAP_INSTANCE_NAME", raising=False)

    providers.get_base_configuration(
        alias=bases.BuilddBaseAlias.JAMMY,
        instance_name="test-instance-name",
    )

    mock_buildd_base.assert_called_with(
        alias=ANY,
        compatibility_tag=ANY,
        environment=ANY,
        hostname=ANY,
        snaps=[bases.buildd.Snap(name="snapcraft", channel=None, classic=True)],
        packages=ANY,
    )


def test_get_command_environment(mocker, mock_default_command_environment):
    """Verify command environment is properly constructed."""
    command_environment = providers.get_command_environment()

    assert command_environment == {"PATH": "test-path", "SNAPCRAFT_MANAGED_MODE": "1"}


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
    monkeypatch.setenv("SNAPCRAFT_MAX_PARALLEL_BUILD_COUNT", "test-build-count")

    # ensure other variables are not being passed
    monkeypatch.setenv("other_var", "test-other-var")

    command_environment = providers.get_command_environment()

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
        "SNAPCRAFT_MAX_PARALLEL_BUILD_COUNT": "test-build-count",
    }


def test_get_command_environment_http_https_proxy(
    mocker, mock_default_command_environment
):
    """Verify http and https proxies are added to the environment."""
    command_environment = providers.get_command_environment(
        http_proxy="test-http", https_proxy="test-https"
    )

    assert command_environment == {
        "PATH": "test-path",
        "SNAPCRAFT_MANAGED_MODE": "1",
        "http_proxy": "test-http",
        "https_proxy": "test-https",
    }


def test_get_command_environment_http_https_priority(
    mocker, mock_default_command_environment, monkeypatch
):
    """Verify http and https proxies from the function argument take priority over the
    proxies defined in the environment."""
    monkeypatch.setenv("http_proxy", "test-http-from-env")
    monkeypatch.setenv("https_proxy", "test-https-from-env")

    command_environment = providers.get_command_environment(
        http_proxy="test-http-from-arg", https_proxy="test-https-from-arg"
    )

    assert command_environment == {
        "PATH": "test-path",
        "SNAPCRAFT_MANAGED_MODE": "1",
        "http_proxy": "test-http-from-arg",
        "https_proxy": "test-https-from-arg",
    }


def test_get_instance_name(new_dir):
    """Test formatting of instance name."""
    inode_number = str(new_dir.stat().st_ino)
    expected_name = f"snapcraft-hello-world-on-arm64-for-armhf-{inode_number}"
    actual_name = providers.get_instance_name(
        project_name="hello-world",
        project_path=new_dir,
        build_on="arm64",
        build_for="armhf",
    )
    assert expected_name == actual_name


@pytest.mark.parametrize(
    "provider, expected_provider",
    [
        ("lxd", LXDProvider),
        ("Lxd", LXDProvider),
        ("LXD", LXDProvider),
        ("multipass", MultipassProvider),
        ("Multipass", MultipassProvider),
        ("MULTIPASS", MultipassProvider),
    ],
)
def test_get_provider_passed_argument(provider, expected_provider):
    """Verify the provider passed into the function argument is chosen."""
    actual_provider = providers.get_provider(provider)

    assert isinstance(actual_provider, expected_provider)


def test_get_provider_passed_argument_invalid():
    """Verify an invalid provider raises an error."""
    with pytest.raises(ValueError) as raised:
        providers.get_provider("invalid-provider")

    assert str(raised.value) == "unsupported provider specified: 'invalid-provider'"


def test_get_provider_passed_argument_priority(mocker, monkeypatch):
    """Verify provider passed into the function argument has the correct priority.

    This takes priority over:
    - environmental variable
    - snap config
    - default
    """
    # set `sys.platform` to verify the default provider 'lxd' is not chosen
    mocker.patch("sys.platform", "linux")

    # set snap config provider to 'lxd' to verify it is not chosen
    mocker.patch(
        "snapcraft.providers.get_snap_config",
        return_value=SnapConfig(provider="lxd"),
    )

    # set the env var to 'lxd' to verify it is not chosen
    monkeypatch.setenv("SNAPCRAFT_BUILD_ENVIRONMENT", "lxd")

    # set function argument to multipass
    provider = providers.get_provider("multipass")

    # provider defined by function argument is chosen
    assert isinstance(provider, MultipassProvider)


@pytest.mark.parametrize(
    "provider, expected_provider",
    [
        ("lxd", LXDProvider),
        ("Lxd", LXDProvider),
        ("LXD", LXDProvider),
        ("multipass", MultipassProvider),
        ("Multipass", MultipassProvider),
        ("MULTIPASS", MultipassProvider),
    ],
)
def test_get_provider_env_var(monkeypatch, provider, expected_provider):
    """Verify the provider defined in SNAPCRAFT_BUILD_ENVIRONMENT is chosen."""
    monkeypatch.setenv("SNAPCRAFT_BUILD_ENVIRONMENT", provider)
    actual_provider = providers.get_provider()

    assert isinstance(actual_provider, expected_provider)


def test_get_provider_env_var_invalid(monkeypatch):
    """Verify an invalid environmental variable raises an error."""
    monkeypatch.setenv("SNAPCRAFT_BUILD_ENVIRONMENT", "invalid-provider")
    with pytest.raises(ValueError) as raised:
        providers.get_provider()

    assert str(raised.value) == "unsupported provider specified: 'invalid-provider'"


def test_get_provider_env_var_priority(mocker, monkeypatch):
    """Verify provider defined by SNAPCRAFT_BUILD_ENVIRONMENT has the correct priority.

    This takes priority over:
    - snap config
    - default
    """
    # set `sys.platform` to verify the default provider 'lxd' is not chosen
    mocker.patch("sys.platform", "linux")

    # set snap config provider to 'lxd' to verify it is not chosen
    mocker.patch(
        "snapcraft.providers.get_snap_config",
        return_value=SnapConfig(provider="lxd"),
    )

    # set env var to multipass
    monkeypatch.setenv("SNAPCRAFT_BUILD_ENVIRONMENT", "multipass")

    # provider defined by env var is chosen
    assert isinstance(providers.get_provider(), MultipassProvider)


@pytest.mark.parametrize(
    "provider, expected_provider",
    [
        ("lxd", LXDProvider),
        ("Lxd", LXDProvider),
        ("LXD", LXDProvider),
        ("multipass", MultipassProvider),
        ("Multipass", MultipassProvider),
        ("MULTIPASS", MultipassProvider),
    ],
)
def test_get_provider_snap_config(mocker, provider, expected_provider):
    """Verify the provider defined in snap config is chosen."""
    # set `sys.platform` to verify the default provider 'lxd' is not chosen
    mocker.patch("sys.platform", "linux")

    # set snap config
    mocker.patch(
        "snapcraft.providers.get_snap_config",
        return_value=SnapConfig(provider=provider),
    )
    actual_provider = providers.get_provider()

    assert isinstance(actual_provider, expected_provider)


def test_get_provider_snap_config_invalid(mocker):
    """Verify an invalid environmental variable raises an error."""
    snap_config = SnapConfig()
    snap_config.provider = "invalid-provider"  # type: ignore
    mocker.patch("snapcraft.providers.get_snap_config", return_value=snap_config)

    with pytest.raises(ValueError) as raised:
        providers.get_provider()

    assert str(raised.value) == "unsupported provider specified: 'invalid-provider'"


def test_get_provider_snap_config_priority(mocker):
    """Verify provider defined by SNAPCRAFT_BUILD_ENVIRONMENT has the correct priority.

    This takes priority over the default.
    """
    # set `sys.platform` to verify the default provider 'lxd' is not chosen
    mocker.patch("sys.platform", "linux")

    # set snap config to multipass
    mocker.patch(
        "snapcraft.providers.get_snap_config",
        return_value=SnapConfig(provider="multipass"),
    )

    # provider defined by snap config is chosen
    assert isinstance(providers.get_provider(), MultipassProvider)


@pytest.mark.parametrize(
    "platform, expected_provider",
    [
        ("linux", LXDProvider),
        ("darwin", MultipassProvider),
        ("win32", MultipassProvider),
        ("unknown", MultipassProvider),
    ],
)
def test_get_provider_snap_config_default(mocker, platform, expected_provider):
    """Verify the default provider is chosen."""
    mocker.patch("sys.platform", platform)
    actual_provider = providers.get_provider()

    assert isinstance(actual_provider, expected_provider)


@pytest.mark.parametrize("bind_ssh", [False, True])
def test_prepare_instance(bind_ssh, mock_instance, mocker, tmp_path):
    """Verify instance is properly prepared."""
    providers.prepare_instance(
        instance=mock_instance, host_project_path=tmp_path, bind_ssh=bind_ssh
    )

    mock_instance.mount.assert_has_calls(
        [call(host_source=tmp_path, target=Path("/root/project"))]
    )

    if bind_ssh:
        mock_instance.mount.assert_has_calls(
            [call(host_source=Path().home() / ".ssh", target=Path("/root/.ssh"))]
        )

    mock_instance.push_file_io.assert_called_with(
        content=ANY, destination=Path("/root/.bashrc"), file_mode="644"
    )
