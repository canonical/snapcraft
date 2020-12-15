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
from testtools.matchers import Equals

import snapcraft.cli._options as options
from tests import unit


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
                kwargs=dict(http_proxy="1.1.1.1", https_proxy="1.1.1.1"),
                flags=dict(http_proxy="1.1.1.1", https_proxy="1.1.1.1"),
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
                ),
                flags=dict(
                    http_proxy="1.1.1.1",
                    https_proxy="1.1.1.1",
                    SNAPCRAFT_BUILD_INFO=True,
                    SNAPCRAFT_IMAGE_INFO="{}",
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
                ),
                flags=dict(
                    http_proxy="1.1.1.1",
                    https_proxy="1.1.1.1",
                    SNAPCRAFT_BUILD_INFO=True,
                    SNAPCRAFT_IMAGE_INFO="{}",
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
                ),
                flags=dict(
                    http_proxy="1.1.1.1",
                    https_proxy="1.1.1.1",
                    SNAPCRAFT_BUILD_INFO=True,
                    SNAPCRAFT_IMAGE_INFO="{}",
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

    @mock.patch("snapcraft.cli._options.warning")
    def test_click_warn_sudo(self, warning_mock):
        options._sanity_check_build_provider_flags("host")
        warning_mock.assert_called_once_with(
            "Running with 'sudo' may cause permission errors and is discouraged. Use 'sudo' when cleaning."
        )
