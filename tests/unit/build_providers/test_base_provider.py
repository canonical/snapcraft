# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

from unittest.mock import Mock, call, ANY

import fixtures
from testtools.matchers import Equals

from . import BaseProviderBaseTest
from snapcraft.internal.build_providers._base_provider import (
    Provider,
    _STORE_ASSERTION_KEY,
)
from tests.unit import fixture_setup


class ProviderImpl(Provider):
    def __init__(self, *, project, echoer):
        super().__init__(project=project, echoer=echoer)

        self.run_mock = Mock()
        self.launch_mock = Mock()
        self.mount_mock = Mock()

    def _run(self, command):
        self.run_mock(command)

    def _launch(self):
        self.launch_mock()

    def _mount(self, *, mountpoint: str, dev_or_path: str):
        self.mount_mock(mountpoint=mountpoint, dev_or_path=dev_or_path)

    def _mount_snaps_directory(self):
        self._mount(mountpoint=self._SNAPS_MOUNTPOINT, dev_or_path="snaps-dev")


class BaseProviderTest(BaseProviderBaseTest):
    def test_initialize(self):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)

        self.assertThat(provider.project, Equals(self.project))
        self.assertThat(provider.instance_name, Equals(self.instance_name))
        self.assertThat(provider.project_dir, Equals("project-name"))
        self.assertThat(
            provider.snap_filename,
            Equals("project-name_{}.snap".format(self.project.deb_arch)),
        )

    def test_initialize_snap_filename_with_version(self):
        self.project.info.version = "test-version"

        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)

        self.assertThat(
            provider.snap_filename,
            Equals("project-name_test-version_{}.snap".format(self.project.deb_arch)),
        )

    def test_launch_instance(self):
        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider.launch_instance()

        provider.launch_mock.assert_any_call()
        provider.run_mock.assert_not_called()
        self.echoer_mock.info.assert_called_once_with(
            "Creating a build environment named {!r}".format(self.instance_name)
        )


class BaseProviderProvisionSnapcraftTest(BaseProviderBaseTest):
    def test_setup_snapcraft_non_snap(self):
        fake_snapd = fixture_setup.FakeSnapd()
        self.useFixture(fake_snapd)
        fake_snapd.snaps_result = []
        fake_snapd.find_result = [
            {
                "core": {
                    "id": "2kkitQ",
                    "channels": {
                        "latest/stable": {"confinement": "strict", "revision": "123"}
                    },
                }
            },
            {
                "snapcraft": {
                    "id": "3lljuR",
                    "channels": {
                        "latest/stable": {"confinement": "classic", "revision": "345"}
                    },
                }
            },
        ]

        self.get_assertion_mock.side_effect = [b"fake-assertion-account-store"]

        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)
        provider.setup_snapcraft()

        provider.launch_mock.assert_not_called()
        provider.run_mock.assert_has_calls(
            [
                call(["sudo", "snap", "set", "core", ANY]),
                call(["sudo", "snap", "watch", "--last=auto-refresh"]),
                call(["sudo", "snap", "ack", ANY]),
                call(["sudo", "snap", "install", "core"]),
                call(["sudo", "snap", "install", "--classic", "snapcraft"]),
            ]
        )
        self.echoer_mock.info.assert_has_calls(
            [
                call("Waiting for pending snap auto refreshes."),
                call("Setting up snapcraft in {!r}".format(self.instance_name)),
                call("Setting up core"),
                call("Setting up snapcraft"),
            ]
        )

    def test_setup_snapcraft_with_snap(self):
        self.useFixture(fixtures.EnvironmentVariable("SNAP_NAME", "snapcraft"))
        self.useFixture(fixtures.EnvironmentVariable("SNAP", "/snap/snapcraft/current"))

        fake_snapd = fixture_setup.FakeSnapd()
        self.useFixture(fake_snapd)
        fake_snapd.snaps_result = [
            {
                "name": "core",
                "confinement": "strict",
                "id": "2kkitQ",
                "channel": "stable",
                "revision": "123",
            },
            {
                "name": "snapcraft",
                "confinement": "classic",
                "id": "3lljuR",
                "channel": "edge",
                "revision": "345",
            },
        ]

        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)

        get_assertion_calls = [
            call(
                ["account-key", "public-key-sha3-384={}".format(_STORE_ASSERTION_KEY)]
            ),
            call(["snap-declaration", "snap-name=core"]),
            call(["snap-revision", "snap-revision=123", "snap-id=2kkitQ"]),
            call(["snap-declaration", "snap-name=snapcraft"]),
            call(["snap-revision", "snap-revision=345", "snap-id=3lljuR"]),
        ]

        self.get_assertion_mock.side_effect = [
            b"fake-assertion-account-store",
            b"fake-assertion-declaration-core",
            b"fake-assertion-revision-core-123",
            b"fake-assertion-declaration-snapcraft",
            b"fake-assertion-revision-snapcraft-345",
        ]
        provider.setup_snapcraft()
        self.get_assertion_mock.assert_has_calls(get_assertion_calls)

        provider.launch_mock.assert_not_called()
        provider.run_mock.assert_has_calls(
            [
                call(["sudo", "snap", "set", "core", ANY]),
                call(["sudo", "snap", "watch", "--last=auto-refresh"]),
                call(["sudo", "snap", "ack", ANY]),
                call(["sudo", "snap", "ack", ANY]),
                call(
                    [
                        "sudo",
                        "snap",
                        "install",
                        "/var/cache/snapcraft/snaps/core_123.snap",
                    ]
                ),
                call(["sudo", "snap", "ack", ANY]),
                call(
                    [
                        "sudo",
                        "snap",
                        "install",
                        "--classic",
                        "/var/cache/snapcraft/snaps/snapcraft_345.snap",
                    ]
                ),
            ]
        )
        provider.mount_mock.assert_called_once_with(
            dev_or_path="snaps-dev", mountpoint="/var/cache/snapcraft/snaps"
        )

    def test_setup_snapcraft_with_dangerous_snap(self):
        self.useFixture(fixtures.EnvironmentVariable("SNAP_NAME", "snapcraft"))
        self.useFixture(fixtures.EnvironmentVariable("SNAP", "/snap/snapcraft/current"))

        fake_snapd = fixture_setup.FakeSnapd()
        self.useFixture(fake_snapd)
        fake_snapd.snaps_result = [
            {
                "name": "core",
                "confinement": "strict",
                "id": "2kkitQ",
                "channel": "stable",
                "revision": "123",
            },
            {
                "name": "snapcraft",
                "confinement": "classic",
                "id": "3lljuR",
                "channel": "edge",
                "revision": "x1",
            },
        ]

        provider = ProviderImpl(project=self.project, echoer=self.echoer_mock)

        get_assertion_calls = [
            call(
                ["account-key", "public-key-sha3-384={}".format(_STORE_ASSERTION_KEY)]
            ),
            call(["snap-declaration", "snap-name=core"]),
            call(["snap-revision", "snap-revision=123", "snap-id=2kkitQ"]),
        ]

        self.get_assertion_mock.side_effect = [
            b"fake-assertion-account-store",
            b"fake-assertion-declaration-core",
            b"fake-assertion-revision-core-123",
        ]
        provider.setup_snapcraft()
        self.get_assertion_mock.assert_has_calls(get_assertion_calls)

        provider.launch_mock.assert_not_called()
        provider.run_mock.assert_has_calls(
            [
                call(["sudo", "snap", "set", "core", ANY]),
                call(["sudo", "snap", "watch", "--last=auto-refresh"]),
                call(["sudo", "snap", "ack", ANY]),
                call(["sudo", "snap", "ack", ANY]),
                call(
                    [
                        "sudo",
                        "snap",
                        "install",
                        "/var/cache/snapcraft/snaps/core_123.snap",
                    ]
                ),
                call(
                    [
                        "sudo",
                        "snap",
                        "install",
                        "--dangerous",
                        "--classic",
                        "/var/cache/snapcraft/snaps/snapcraft_x1.snap",
                    ]
                ),
            ]
        )
        provider.mount_mock.assert_called_once_with(
            dev_or_path="snaps-dev", mountpoint="/var/cache/snapcraft/snaps"
        )
