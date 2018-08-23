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

import os
from textwrap import dedent
from unittest.mock import call, patch, ANY

from testtools.matchers import FileContains

from . import ProviderImpl, get_project
from snapcraft.internal.build_providers._snap import SnapInjector, repo
from tests import fixture_setup, unit


class SnapInjectionTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        patcher = patch("snapcraft.internal.repo.snaps.get_assertion")
        self.get_assertion_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.fake_snapd = fixture_setup.FakeSnapd()
        self.useFixture(self.fake_snapd)

        self.registry_filepath = os.path.join(self.path, "registry.yaml")

        self.provider = ProviderImpl(project=get_project(), echoer=lambda x: x)

    def test_snapcraft_installed_on_host_from_store(self):
        self.fake_snapd.snaps_result = [
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
        self.get_assertion_mock.side_effect = [
            b"fake-assertion-account-store",
            b"fake-assertion-declaration-core",
            b"fake-assertion-revision-core-123",
            b"fake-assertion-declaration-snapcraft",
            b"fake-assertion-revision-snapcraft-345",
        ]

        snap_injector = SnapInjector(
            snap_dir=self.provider._SNAPS_MOUNTPOINT,
            registry_filepath=self.registry_filepath,
            snap_arch="amd64",
            runner=self.provider._run,
            snap_dir_mounter=self.provider._mount_snaps_directory,
            snap_dir_unmounter=self.provider._unmount_snaps_directory,
            file_pusher=self.provider._push_file,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")
        snap_injector.apply()

        get_assertion_calls = [
            call(
                [
                    "account-key",
                    "public-key-sha3-384=BWDEoaqyr25nF5SNCvEv2v7QnM9QsfCc0PBMYD_i2NGSQ32EF2d4D0hqUel3m8ul",
                ]
            ),
            call(["snap-declaration", "snap-name=core"]),
            call(["snap-revision", "snap-revision=123", "snap-id=2kkitQ"]),
            call(["snap-declaration", "snap-name=snapcraft"]),
            call(["snap-revision", "snap-revision=345", "snap-id=3lljuR"]),
        ]
        self.get_assertion_mock.assert_has_calls(get_assertion_calls)
        self.provider.run_mock.assert_has_calls(
            [
                call(["sudo", "snap", "set", "core", ANY]),
                call(["sudo", "snap", "watch", "--last=auto-refresh"]),
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
                        "--classic",
                        "/var/cache/snapcraft/snaps/snapcraft_345.snap",
                    ]
                ),
            ]
        )
        self.provider.mount_mock.assert_called_once_with(
            dev_or_path="snaps-dev", mountpoint="/var/cache/snapcraft/snaps"
        )
        self.provider.unmount_mock.assert_called_once_with(
            mountpoint="/var/cache/snapcraft/snaps"
        )
        self.provider.push_file_mock.assert_called_once_with(
            source=ANY, destination=ANY
        )
        self.assertThat(
            self.registry_filepath,
            FileContains(
                dedent(
                    """\
                    core:
                    - {revision: '123'}
                    snapcraft:
                    - {revision: '345'}
                    """
                )
            ),
        )

    def test_snapcraft_installed_on_host_from_store_but_injection_disabled(self):
        self.useFixture(fixture_setup.FakeStore())

        snap_injector = SnapInjector(
            snap_dir=self.provider._SNAPS_MOUNTPOINT,
            registry_filepath=self.registry_filepath,
            snap_arch="amd64",
            runner=self.provider._run,
            snap_dir_mounter=self.provider._mount_snaps_directory,
            snap_dir_unmounter=self.provider._unmount_snaps_directory,
            file_pusher=self.provider._push_file,
            inject_from_host=False,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")
        snap_injector.apply()

        self.get_assertion_mock.assert_not_called()
        self.provider.run_mock.assert_has_calls(
            [
                call(["sudo", "snap", "set", "core", ANY]),
                call(["sudo", "snap", "watch", "--last=auto-refresh"]),
                call(["sudo", "snap", "install", "core"]),
                call(["sudo", "snap", "install", "--classic", "snapcraft"]),
            ]
        )
        self.provider.mount_mock.assert_not_called()
        self.provider.unmount_mock.assert_not_called()
        self.provider.push_file_mock.assert_not_called()
        self.assertThat(
            self.registry_filepath,
            FileContains(
                dedent(
                    """\
                    core:
                    - {revision: '10000'}
                    snapcraft:
                    - {revision: '25'}
                    """
                )
            ),
        )

    def test_snapcraft_installed_on_host_with_dangerous(self):
        self.fake_snapd.snaps_result = [
            {
                "name": "core",
                "confinement": "strict",
                "id": "2kkitQ",
                "channel": "stable",
                "revision": "123",
            },
            {"name": "snapcraft", "confinement": "classic", "revision": "x20"},
        ]
        self.get_assertion_mock.side_effect = [
            b"fake-assertion-account-store",
            b"fake-assertion-declaration-core",
            b"fake-assertion-revision-core-123",
        ]

        snap_injector = SnapInjector(
            snap_dir=self.provider._SNAPS_MOUNTPOINT,
            registry_filepath=self.registry_filepath,
            snap_arch="amd64",
            runner=self.provider._run,
            snap_dir_mounter=self.provider._mount_snaps_directory,
            snap_dir_unmounter=self.provider._unmount_snaps_directory,
            file_pusher=self.provider._push_file,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")
        snap_injector.apply()

        get_assertion_calls = [
            call(
                [
                    "account-key",
                    "public-key-sha3-384=BWDEoaqyr25nF5SNCvEv2v7QnM9QsfCc0PBMYD_i2NGSQ32EF2d4D0hqUel3m8ul",
                ]
            ),
            call(["snap-declaration", "snap-name=core"]),
            call(["snap-revision", "snap-revision=123", "snap-id=2kkitQ"]),
        ]
        self.get_assertion_mock.assert_has_calls(get_assertion_calls)
        self.provider.run_mock.assert_has_calls(
            [
                call(["sudo", "snap", "set", "core", ANY]),
                call(["sudo", "snap", "watch", "--last=auto-refresh"]),
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
                        "/var/cache/snapcraft/snaps/snapcraft_x20.snap",
                    ]
                ),
            ]
        )
        self.provider.mount_mock.assert_called_once_with(
            dev_or_path="snaps-dev", mountpoint="/var/cache/snapcraft/snaps"
        )
        self.provider.unmount_mock.assert_called_once_with(
            mountpoint="/var/cache/snapcraft/snaps"
        )
        self.provider.push_file_mock.assert_called_once_with(
            source=ANY, destination=ANY
        )
        self.assertThat(
            self.registry_filepath,
            FileContains(
                dedent(
                    """\
                    core:
                    - {revision: '123'}
                    snapcraft:
                    - {revision: x20}
                    """
                )
            ),
        )

    def test_snapcraft_not_installed_on_host(self):
        self.useFixture(fixture_setup.FakeStore())

        snap_injector = SnapInjector(
            snap_dir=self.provider._SNAPS_MOUNTPOINT,
            registry_filepath=self.registry_filepath,
            snap_arch="amd64",
            runner=self.provider._run,
            snap_dir_mounter=self.provider._mount_snaps_directory,
            snap_dir_unmounter=self.provider._unmount_snaps_directory,
            file_pusher=self.provider._push_file,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")
        snap_injector.apply()

        self.get_assertion_mock.assert_not_called()
        self.provider.run_mock.assert_has_calls(
            [
                call(["sudo", "snap", "set", "core", ANY]),
                call(["sudo", "snap", "watch", "--last=auto-refresh"]),
                call(["sudo", "snap", "install", "core"]),
                call(["sudo", "snap", "install", "--classic", "snapcraft"]),
            ]
        )
        self.provider.mount_mock.assert_not_called()
        self.provider.unmount_mock.assert_not_called()
        self.provider.push_file_mock.assert_not_called()
        self.assertThat(
            self.registry_filepath,
            FileContains(
                dedent(
                    """\
                    core:
                    - {revision: '10000'}
                    snapcraft:
                    - {revision: '25'}
                    """
                )
            ),
        )

    def test_no_registry(self):
        self.useFixture(fixture_setup.FakeStore())

        snap_injector = SnapInjector(
            snap_dir=self.provider._SNAPS_MOUNTPOINT,
            registry_filepath=None,
            snap_arch="amd64",
            runner=self.provider._run,
            snap_dir_mounter=self.provider._mount_snaps_directory,
            snap_dir_unmounter=self.provider._unmount_snaps_directory,
            file_pusher=self.provider._push_file,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")
        snap_injector.apply()

        self.provider.run_mock.assert_has_calls(
            [
                call(["sudo", "snap", "set", "core", ANY]),
                call(["sudo", "snap", "watch", "--last=auto-refresh"]),
                call(["sudo", "snap", "install", "core"]),
                call(["sudo", "snap", "install", "--classic", "snapcraft"]),
            ]
        )
        self.provider.run_mock.reset_mock()

        snap_injector = SnapInjector(
            snap_dir=self.provider._SNAPS_MOUNTPOINT,
            registry_filepath=self.registry_filepath,
            snap_arch="amd64",
            runner=self.provider._run,
            snap_dir_mounter=self.provider._mount_snaps_directory,
            snap_dir_unmounter=self.provider._unmount_snaps_directory,
            file_pusher=self.provider._push_file,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")
        snap_injector.apply()

        self.provider.run_mock.assert_has_calls(
            [
                call(["sudo", "snap", "set", "core", ANY]),
                call(["sudo", "snap", "watch", "--last=auto-refresh"]),
                call(["sudo", "snap", "install", "core"]),
                call(["sudo", "snap", "install", "--classic", "snapcraft"]),
            ]
        )

    def test_snapcraft_installed_on_host_from_store_rerun_is_nop(self):
        self.fake_snapd.snaps_result = [
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
        self.get_assertion_mock.side_effect = [
            b"fake-assertion-account-store",
            b"fake-assertion-declaration-core",
            b"fake-assertion-revision-core-123",
            b"fake-assertion-declaration-snapcraft",
            b"fake-assertion-revision-snapcraft-345",
        ]

        snap_injector = SnapInjector(
            snap_dir=self.provider._SNAPS_MOUNTPOINT,
            registry_filepath=self.registry_filepath,
            snap_arch="amd64",
            runner=self.provider._run,
            snap_dir_mounter=self.provider._mount_snaps_directory,
            snap_dir_unmounter=self.provider._unmount_snaps_directory,
            file_pusher=self.provider._push_file,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")
        snap_injector.apply()
        self.provider.run_mock.reset_mock()

        snap_injector = SnapInjector(
            snap_dir=self.provider._SNAPS_MOUNTPOINT,
            registry_filepath=self.registry_filepath,
            snap_arch="amd64",
            runner=self.provider._run,
            snap_dir_mounter=self.provider._mount_snaps_directory,
            snap_dir_unmounter=self.provider._unmount_snaps_directory,
            file_pusher=self.provider._push_file,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")
        snap_injector.apply()

        self.provider.run_mock.assert_not_called()

    def test_snapcraft_installed_on_host_from_store_rerun_refreshes(self):
        self.useFixture(fixture_setup.FakeStore())

        snap_injector = SnapInjector(
            snap_dir=self.provider._SNAPS_MOUNTPOINT,
            registry_filepath=self.registry_filepath,
            snap_arch="amd64",
            runner=self.provider._run,
            snap_dir_mounter=self.provider._mount_snaps_directory,
            snap_dir_unmounter=self.provider._unmount_snaps_directory,
            file_pusher=self.provider._push_file,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")
        snap_injector.apply()
        self.provider.run_mock.reset_mock()

        snap_injector = SnapInjector(
            snap_dir=self.provider._SNAPS_MOUNTPOINT,
            registry_filepath=self.registry_filepath,
            snap_arch="amd64",
            runner=self.provider._run,
            snap_dir_mounter=self.provider._mount_snaps_directory,
            snap_dir_unmounter=self.provider._unmount_snaps_directory,
            file_pusher=self.provider._push_file,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")
        snap_injector.apply()

        self.provider.run_mock.assert_has_calls(
            [
                call(["sudo", "snap", "set", "core", ANY]),
                call(["sudo", "snap", "watch", "--last=auto-refresh"]),
                call(["sudo", "snap", "refresh", "core"]),
                call(["sudo", "snap", "refresh", "--classic", "snapcraft"]),
            ]
        )

    def test_snapd_not_on_host_installs_from_store(self):
        self.useFixture(fixture_setup.FakeStore())

        snap_injector = SnapInjector(
            snap_dir=self.provider._SNAPS_MOUNTPOINT,
            registry_filepath=self.registry_filepath,
            snap_arch="amd64",
            runner=self.provider._run,
            snap_dir_mounter=self.provider._mount_snaps_directory,
            snap_dir_unmounter=self.provider._unmount_snaps_directory,
            file_pusher=self.provider._push_file,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")

        with patch(
            "snapcraft.internal.repo.snaps.SnapPackage.get_local_snap_info",
            side_effect=repo.errors.SnapdConnectionError("core", "url"),
        ):
            snap_injector.apply()

        self.get_assertion_mock.assert_not_called()
        self.provider.run_mock.assert_has_calls(
            [
                call(["sudo", "snap", "set", "core", ANY]),
                call(["sudo", "snap", "watch", "--last=auto-refresh"]),
                call(["sudo", "snap", "install", "core"]),
                call(["sudo", "snap", "install", "--classic", "snapcraft"]),
            ]
        )
        self.provider.mount_mock.assert_not_called()
        self.provider.unmount_mock.assert_not_called()
        self.provider.push_file_mock.assert_not_called()
        self.assertThat(
            self.registry_filepath,
            FileContains(
                dedent(
                    """\
                    core:
                    - {revision: '10000'}
                    snapcraft:
                    - {revision: '25'}
                    """
                )
            ),
        )
