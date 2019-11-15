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

import click
from unittest import mock

import fixtures
from testtools.matchers import Equals

import snapcraft.cli._options as options
from tests import unit


class TestProviderOptions(unit.TestCase):
    scenarios = [
        ("host empty", dict(provider="host", kwargs=dict())),
        ("host http proxy", dict(provider="host", kwargs=dict(http_proxy="1.1.1.1"))),
        ("host https proxy", dict(provider="host", kwargs=dict(https_proxy="1.1.1.1"))),
        (
            "host all",
            dict(
                provider="host",
                kwargs=dict(http_proxy="1.1.1.1", https_proxy="1.1.1.1"),
            ),
        ),
        ("lxd empty", dict(provider="lxd", kwargs=dict())),
        (
            "lxd apt mirror",
            dict(
                provider="lxd",
                kwargs=dict(apt_mirror="http://de.archive.ubuntu.com/ubuntu"),
            ),
        ),
        ("lxd bind ssh true", dict(provider="lxd", kwargs=dict(bind_ssh=True))),
        ("lxd bind ssh false", dict(provider="lxd", kwargs=dict(bind_ssh=False))),
        ("lxd http proxy", dict(provider="lxd", kwargs=dict(http_proxy="1.1.1.1"))),
        ("lxd https proxy", dict(provider="lxd", kwargs=dict(https_proxy="1.1.1.1"))),
        (
            "lxd all",
            dict(
                provider="lxd",
                kwargs=dict(
                    apt_mirror="http://de.archive.ubuntu.com/ubuntu",
                    bind_ssh=True,
                    http_proxy="1.1.1.1",
                    https_proxy="1.1.1.1",
                ),
            ),
        ),
        ("managed-host empty", dict(provider="managed-host", kwargs=dict())),
        (
            "managed-host http proxy",
            dict(provider="managed-host", kwargs=dict(http_proxy="1.1.1.1")),
        ),
        (
            "managed-host https proxy",
            dict(provider="managed-host", kwargs=dict(https_proxy="1.1.1.1")),
        ),
        (
            "managed-host all",
            dict(
                provider="managed-host",
                kwargs=dict(http_proxy="1.1.1.1", https_proxy="1.1.1.1"),
            ),
        ),
        ("multipass empty", dict(provider="multipass", kwargs=dict())),
        (
            "multipass apt mirror",
            dict(
                provider="multipass",
                kwargs=dict(apt_mirror="http://de.archive.ubuntu.com/ubuntu"),
            ),
        ),
        (
            "multipass bind ssh true",
            dict(provider="multipass", kwargs=dict(bind_ssh=True)),
        ),
        (
            "multipass bind ssh false",
            dict(provider="multipass", kwargs=dict(bind_ssh=False)),
        ),
        (
            "multipass http proxy",
            dict(provider="multipass", kwargs=dict(http_proxy="1.1.1.1")),
        ),
        (
            "multipass https proxy",
            dict(provider="multipass", kwargs=dict(https_proxy="1.1.1.1")),
        ),
        (
            "multipass all",
            dict(
                provider="multipass",
                kwargs=dict(
                    apt_mirror="http://de.archive.ubuntu.com/ubuntu",
                    bind_ssh=True,
                    http_proxy="1.1.1.1",
                    https_proxy="1.1.1.1",
                ),
            ),
        ),
    ]

    def test_valid_flags(self):
        flags = options.get_build_provider_flags(self.provider, **self.kwargs)

        self.assertThat(flags, Equals(self.kwargs))


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
