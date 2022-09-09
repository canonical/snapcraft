# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2019 Canonical Ltd
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

from unittest import mock

import click
import fixtures
import pytest
from testtools.matchers import Equals

import snapcraft_legacy.cli._options as options
from tests.legacy import unit


class TestProviderOptions:
    scenarios = [
        ("host empty", dict(provider="host", kwargs=dict(), flags=dict())),
        (
            "host http proxy",
            dict(
                provider="host",
                kwargs=dict(http_proxy="1.1.1.1"),
                flags=dict(http_proxy="1.1.1.1"),
            ),
        ),
        (
            "host https proxy",
            dict(
                provider="host",
                kwargs=dict(https_proxy="1.1.1.1"),
                flags=dict(https_proxy="1.1.1.1"),
            ),
        ),
        (
            "host build info",
            dict(
                provider="host",
                kwargs=dict(enable_manifest=True),
                flags=dict(SNAPCRAFT_BUILD_INFO=True),
            ),
        ),
        (
            "host build info off",
            dict(provider="host", kwargs=dict(enable_manifest=False), flags=dict()),
        ),
        (
            "host image info",
            dict(
                provider="host",
                kwargs=dict(manifest_image_information="{}"),
                flags=dict(SNAPCRAFT_IMAGE_INFO="{}"),
            ),
        ),
        (
            "host all",
            dict(
                provider="host",
                kwargs=dict(
                    http_proxy="1.1.1.1",
                    https_proxy="1.1.1.1",
                    offline=True,
                ),
                flags=dict(
                    http_proxy="1.1.1.1", https_proxy="1.1.1.1", SNAPCRAFT_OFFLINE=True
                ),
            ),
        ),
        ("lxd empty", dict(provider="lxd", kwargs=dict(), flags=dict())),
        (
            "lxd http proxy",
            dict(
                provider="lxd",
                kwargs=dict(http_proxy="1.1.1.1"),
                flags=dict(http_proxy="1.1.1.1"),
            ),
        ),
        (
            "lxd https proxy",
            dict(
                provider="lxd",
                kwargs=dict(https_proxy="1.1.1.1"),
                flags=dict(https_proxy="1.1.1.1"),
            ),
        ),
        (
            "lxd build info",
            dict(
                provider="lxd",
                kwargs=dict(enable_manifest=True),
                flags=dict(SNAPCRAFT_BUILD_INFO=True),
            ),
        ),
        (
            "lxd image info",
            dict(
                provider="lxd",
                kwargs=dict(manifest_image_information="{}"),
                flags=dict(SNAPCRAFT_IMAGE_INFO="{}"),
            ),
        ),
        (
            "lxd all",
            dict(
                provider="lxd",
                kwargs=dict(
                    http_proxy="1.1.1.1",
                    https_proxy="1.1.1.1",
                    enable_manifest=True,
                    manifest_image_information="{}",
                    offline=True,
                ),
                flags=dict(
                    http_proxy="1.1.1.1",
                    https_proxy="1.1.1.1",
                    SNAPCRAFT_BUILD_INFO=True,
                    SNAPCRAFT_IMAGE_INFO="{}",
                    SNAPCRAFT_OFFLINE=True,
                ),
            ),
        ),
        (
            "managed-host empty",
            dict(provider="managed-host", kwargs=dict(), flags=dict()),
        ),
        (
            "managed-host http proxy",
            dict(
                provider="managed-host",
                kwargs=dict(http_proxy="1.1.1.1"),
                flags=dict(http_proxy="1.1.1.1"),
            ),
        ),
        (
            "managed-host https proxy",
            dict(
                provider="managed-host",
                kwargs=dict(https_proxy="1.1.1.1"),
                flags=dict(https_proxy="1.1.1.1"),
            ),
        ),
        (
            "managed-host build info",
            dict(
                provider="managed-host",
                kwargs=dict(enable_manifest=True),
                flags=dict(SNAPCRAFT_BUILD_INFO=True),
            ),
        ),
        (
            "managed-host image info",
            dict(
                provider="managed-host",
                kwargs=dict(manifest_image_information="{}"),
                flags=dict(SNAPCRAFT_IMAGE_INFO="{}"),
            ),
        ),
        (
            "managed-host all",
            dict(
                provider="managed-host",
                kwargs=dict(
                    http_proxy="1.1.1.1",
                    https_proxy="1.1.1.1",
                    enable_manifest=True,
                    manifest_image_information="{}",
                    offline=True,
                ),
                flags=dict(
                    http_proxy="1.1.1.1",
                    https_proxy="1.1.1.1",
                    SNAPCRAFT_BUILD_INFO=True,
                    SNAPCRAFT_IMAGE_INFO="{}",
                    SNAPCRAFT_OFFLINE=True,
                ),
            ),
        ),
        ("multipass empty", dict(provider="multipass", kwargs=dict(), flags=dict())),
        (
            "multipass http proxy",
            dict(
                provider="multipass",
                kwargs=dict(http_proxy="1.1.1.1"),
                flags=dict(http_proxy="1.1.1.1"),
            ),
        ),
        (
            "multipass https proxy",
            dict(
                provider="multipass",
                kwargs=dict(https_proxy="1.1.1.1"),
                flags=dict(https_proxy="1.1.1.1"),
            ),
        ),
        (
            "multipass build info",
            dict(
                provider="multipass",
                kwargs=dict(enable_manifest=True),
                flags=dict(SNAPCRAFT_BUILD_INFO=True),
            ),
        ),
        (
            "multipass image info",
            dict(
                provider="multipass",
                kwargs=dict(manifest_image_information="{}"),
                flags=dict(SNAPCRAFT_IMAGE_INFO="{}"),
            ),
        ),
        (
            "multipass all",
            dict(
                provider="multipass",
                kwargs=dict(
                    http_proxy="1.1.1.1",
                    https_proxy="1.1.1.1",
                    enable_manifest=True,
                    manifest_image_information="{}",
                    offline=True,
                ),
                flags=dict(
                    http_proxy="1.1.1.1",
                    https_proxy="1.1.1.1",
                    SNAPCRAFT_BUILD_INFO=True,
                    SNAPCRAFT_IMAGE_INFO="{}",
                    SNAPCRAFT_OFFLINE=True,
                ),
            ),
        ),
    ]

    def test(self, provider, kwargs, flags):
        assert options.get_build_provider_flags(provider, **kwargs) == flags


class TestInvalidBuildProviderFlags(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT", None)
        )

    def test_ignored(self):
        kwargs = dict(notrelevant="foo")

        flags = options.get_build_provider_flags("lxd", **kwargs)

        self.assertThat(flags, Equals(dict()))

    def test_host_missing_destructive_mode(self):
        kwargs = dict(http_proxy="192.168.1.1")

        self.assertRaisesRegex(
            click.exceptions.BadArgumentUsage,
            "^--provider=host requires --destructive-mode to acknowledge side effects$",
            options._sanity_check_build_provider_flags,
            "host",
            **kwargs,
        )

    def test_build_environment_mismatch(self):
        kwargs = dict(http_proxy="192.168.1.1")

        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT", "managed-host")
        )

        self.assertRaisesRegex(
            click.exceptions.BadArgumentUsage,
            "^mismatch between --provider=lxd and SNAPCRAFT_BUILD_ENVIRONMENT=managed-host$",
            options._sanity_check_build_provider_flags,
            "lxd",
            **kwargs,
        )

    def test_wrong_provider_option_error(self):
        kwargs = dict(http_proxy="192.168.1.1")
        fake_args = ["--destructive-mode"]

        with mock.patch("sys.argv", fake_args):
            self.assertRaisesRegex(
                click.exceptions.BadArgumentUsage,
                "^--destructive-mode cannot be used with build provider 'lxd'$",
                options._sanity_check_build_provider_flags,
                "lxd",
                **kwargs,
            )

    def test_unknown_provider(self):
        kwargs = dict(http_proxy="192.168.1.1")
        fake_args = ["--http-proxy", "192.168.1.1"]

        with mock.patch("sys.argv", fake_args):
            self.assertRaisesRegex(
                click.exceptions.BadArgumentUsage,
                "^--http-proxy cannot be used with build provider 'invalid-provider'$",
                options._sanity_check_build_provider_flags,
                "invalid-provider",
                **kwargs,
            )


class TestSudo(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.useFixture(
            fixtures.EnvironmentVariable("SNAPCRAFT_BUILD_ENVIRONMENT", "host")
        )
        self.useFixture(fixtures.EnvironmentVariable("SUDO_USER", "testuser"))
        self.fake_euid = self.useFixture(
            fixtures.MockPatch("os.geteuid", return_value=0)
        ).mock

    @mock.patch("snapcraft_legacy.cli._options.warning")
    def test_click_warn_sudo(self, warning_mock):
        options._sanity_check_build_provider_flags("host")
        warning_mock.assert_called_once_with(
            "Running with 'sudo' may cause permission errors and is discouraged. Use 'sudo' when cleaning."
        )


@pytest.mark.parametrize("provider_name", ["host", "lxd", "multipass", "managed-host"])
def test_get_build_provider_passed_argument(mocker, provider_name):
    """Verify the provider specified by --provider=<provider>.

    This takes priority over:
    - `-use-lxd`
    - `--destructive-mode`
    - `is_process_container()`
    - snap config
    - default
    """
    # set snap config provider to 'multipass' to verify it is not chosen
    mocker.patch(
        "snapcraft_legacy.cli._options.common.get_snap_config",
        return_value={"provider": "multipass"},
    )
    # return True from `is_process_container()` to verify 'host' is not chosen
    mocker.patch(
        "snapcraft_legacy.cli._options.common.is_process_container", return_value=True
    )
    # set destructive_mode to True to verify 'host' is not chosen
    kwargs = dict(provider=provider_name, destructive_mode=True)
    actual_provider_name = options.get_build_provider(**kwargs)

    assert actual_provider_name == provider_name


def test_get_build_provider_passed_argument_invalid(mocker):
    """Verify an invalid provider raises an error."""
    with pytest.raises(ValueError) as raised:
        options.get_build_provider(provider="invalid-provider")

    assert str(raised.value) == "unsupported provider specified: 'invalid-provider'"


def test_get_build_provider_use_lxd(mocker):
    """Verify 'lxd' is chosen when `--use-lxd` is passed.

    This takes priority over:
    - `--destructive-mode`
    - `is_process_container()`
    - snap config
    - default
    """
    # set snap config provider to 'multipass' to verify it is not chosen
    mocker.patch(
        "snapcraft_legacy.cli._options.common.get_snap_config",
        return_value={"provider": "multipass"},
    )
    # `is_process_container()` returns True to verify 'host' is not chosen
    mocker.patch(
        "snapcraft_legacy.cli._options.common.is_process_container", return_value=True
    )
    # set destructive_mode to True to verify 'host' is not chosen
    kwargs = dict(use_lxd=True, destructive_mode=True)
    provider_name = options.get_build_provider(**kwargs)

    assert provider_name == "lxd"


def test_get_build_provider_destructive_mode(mocker, monkeypatch):
    """Verify 'host' is chosen when `--destructive-mode` is passed.

    This takes priority over:
    - `is_process_container()`
    - snap config
    - default
    """
    # set snap config provider to 'multipass' to verify it is not chosen
    mocker.patch(
        "snapcraft_legacy.cli._options.common.get_snap_config",
        return_value={"provider": "multipass"},
    )

    kwargs = dict(destructive_mode=True)
    provider_name = options.get_build_provider(**kwargs)

    assert provider_name == "host"


def test_get_build_provider_is_process_container(mocker):
    """Verify 'host' is chosen when snapcraft is running as a process container.

    This takes priority over:
    - snap config
    - default
    """
    # set snap config provider to 'multipass' to verify it is not chosen
    mocker.patch(
        "snapcraft_legacy.cli._options.common.get_snap_config",
        return_value={"provider": "multipass"},
    )
    # is_process_container returns True
    mocker.patch(
        "snapcraft_legacy.cli._options.common.is_process_container", return_value=True
    )
    provider_name = options.get_build_provider()

    assert provider_name == "host"


@pytest.mark.parametrize(
    "provider_name, expected_provider_name",
    [
        ("lxd", "lxd"),
        ("Lxd", "lxd"),
        ("LXD", "lxd"),
        ("multipass", "multipass"),
        ("Multipass", "multipass"),
        ("MULTIPASS", "multipass"),
    ],
)
def test_get_build_provider_snap_config(mocker, provider_name, expected_provider_name):
    """Verify the provider specified by the snap config is chosen.

    Additionally verify that the value parsed from the snap config is converted to lowercase.

    This takes priority over:
    - default
    """
    mocker.patch(
        "snapcraft_legacy.cli._options.common.get_snap_config",
        return_value={"provider": provider_name},
    )
    actual_provider_name = options.get_build_provider()

    assert actual_provider_name == expected_provider_name


def test_get_build_provider_snap_config_invalid(mocker):
    """Verify an invalid provider from the snap config raises an error."""
    mocker.patch(
        "snapcraft_legacy.cli._options.common.get_snap_config",
        return_value={"provider": "invalid-provider"},
    )

    with pytest.raises(ValueError) as raised:
        options.get_build_provider()

    assert str(raised.value) == "unsupported provider specified: 'invalid-provider'"


def test_get_build_provider_default(mocker):
    """Verify the default provider 'multipass' is chosen."""
    mocker.patch(
        "snapcraft_legacy.cli._options.common.get_snap_config", return_value=None
    )
    mocker.patch(
        "snapcraft_legacy.cli._options.common.is_process_container", return_value=False
    )
    provider_name = options.get_build_provider()

    assert provider_name == "multipass"
