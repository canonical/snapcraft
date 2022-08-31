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

import pytest

from snapcraft.providers import LXDProvider, MultipassProvider, get_provider
from snapcraft.snap_config import SnapConfig


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
    actual_provider = get_provider(provider)

    assert isinstance(actual_provider, expected_provider)


def test_get_provider_passed_argument_invalid():
    """Verify an invalid provider raises an error."""
    with pytest.raises(RuntimeError) as raised:
        get_provider("invalid-provider")

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
        "snapcraft.providers._get_provider.get_snap_config",
        return_value=SnapConfig(provider="lxd"),
    )

    # set the env var to 'lxd' to verify it is not chosen
    monkeypatch.setenv("SNAPCRAFT_BUILD_ENVIRONMENT", "lxd")

    # set function argument to multipass
    provider = get_provider("multipass")

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
    actual_provider = get_provider()

    assert isinstance(actual_provider, expected_provider)


def test_get_provider_env_var_invalid(monkeypatch):
    """Verify an invalid environmental variable raises an error."""
    monkeypatch.setenv("SNAPCRAFT_BUILD_ENVIRONMENT", "invalid-provider")
    with pytest.raises(RuntimeError) as raised:
        get_provider()

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
        "snapcraft.providers._get_provider.get_snap_config",
        return_value=SnapConfig(provider="lxd"),
    )

    # set env var to multipass
    monkeypatch.setenv("SNAPCRAFT_BUILD_ENVIRONMENT", "multipass")

    # provider defined by env var is chosen
    assert isinstance(get_provider(), MultipassProvider)


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
        "snapcraft.providers._get_provider.get_snap_config",
        return_value=SnapConfig(provider=provider),
    )
    actual_provider = get_provider()

    assert isinstance(actual_provider, expected_provider)


def test_get_provider_snap_config_invalid(mocker):
    """Verify an invalid environmental variable raises an error."""
    snap_config = SnapConfig()
    snap_config.provider = "invalid-provider"  # type: ignore
    mocker.patch(
        "snapcraft.providers._get_provider.get_snap_config", return_value=snap_config
    )

    with pytest.raises(RuntimeError) as raised:
        get_provider()

    assert str(raised.value) == "unsupported provider specified: 'invalid-provider'"


def test_get_provider_snap_config_priority(mocker):
    """Verify provider defined by SNAPCRAFT_BUILD_ENVIRONMENT has the correct priority.

    This takes priority over the default.
    """
    # set `sys.platform` to verify the default provider 'lxd' is not chosen
    mocker.patch("sys.platform", "linux")

    # set snap config to multipass
    mocker.patch(
        "snapcraft.providers._get_provider.get_snap_config",
        return_value=SnapConfig(provider="multipass"),
    )

    # provider defined by snap config is chosen
    assert isinstance(get_provider(), MultipassProvider)


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
    actual_provider = get_provider()

    assert isinstance(actual_provider, expected_provider)
