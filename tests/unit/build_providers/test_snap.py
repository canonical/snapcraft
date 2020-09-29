# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2019 Canonical Ltd
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

import logging
import os
from textwrap import dedent
from unittest.mock import ANY, call, patch

import fixtures
from testtools.matchers import Contains, Equals, FileContains, Not

from snapcraft.internal.build_providers._snap import (
    SnapInjector,
    _get_snap_channel,
    repo,
)
from tests import fixture_setup, unit

from . import ProviderImpl, get_project


class SnapInjectionTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        patcher = patch("snapcraft.internal.repo.snaps.get_assertion")
        self.get_assertion_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.registry_filepath = os.path.join(self.path, "registry.yaml")

        self.provider = ProviderImpl(project=get_project(), echoer=lambda x: x)

    def test_snapcraft_installed_on_host_from_store(self):
        self.fake_snapd.snaps_result = [
            {
                "name": "snapd",
                "confinement": "strict",
                "id": "2kkitQ",
                "channel": "edge",
                "revision": "1",
                "tracking-channel": "latest/edge",
            },
            {
                "name": "core18",
                "confinement": "strict",
                "id": "2kkibb",
                "channel": "stable",
                "revision": "123",
                "tracking-channel": "latest/beta",
            },
            {
                "name": "snapcraft",
                "confinement": "classic",
                "id": "3lljuR",
                "channel": "edge",
                "revision": "345",
                "tracking-channel": "latest/candidate",
            },
        ]
        self.get_assertion_mock.side_effect = [
            b"fake-assertion-account-store",
            b"fake-assertion-declaration-snapd",
            b"fake-assertion-revision-snapd-1",
            b"fake-assertion-account-store",
            b"fake-assertion-declaration-core18",
            b"fake-assertion-revision-core18-123",
            b"fake-assertion-account-store",
            b"fake-assertion-declaration-snapcraft",
            b"fake-assertion-revision-snapcraft-345",
        ]

        snap_injector = SnapInjector(
            registry_filepath=self.registry_filepath,
            runner=self.provider._run,
            file_pusher=self.provider._push_file,
        )
        snap_injector.add("snapd")
        snap_injector.add("core18")
        snap_injector.add("snapcraft")
        snap_injector.apply()

        get_assertion_calls = [
            call(
                [
                    "account-key",
                    "public-key-sha3-384=BWDEoaqyr25nF5SNCvEv2v7QnM9QsfCc0PBMYD_i2NGSQ32EF2d4D0hqUel3m8ul",
                ]
            ),
            call(["snap-declaration", "snap-name=snapd"]),
            call(["snap-revision", "snap-revision=1", "snap-id=2kkitQ"]),
            call(
                [
                    "account-key",
                    "public-key-sha3-384=BWDEoaqyr25nF5SNCvEv2v7QnM9QsfCc0PBMYD_i2NGSQ32EF2d4D0hqUel3m8ul",
                ]
            ),
            call(["snap-declaration", "snap-name=core18"]),
            call(["snap-revision", "snap-revision=123", "snap-id=2kkibb"]),
            call(
                [
                    "account-key",
                    "public-key-sha3-384=BWDEoaqyr25nF5SNCvEv2v7QnM9QsfCc0PBMYD_i2NGSQ32EF2d4D0hqUel3m8ul",
                ]
            ),
            call(["snap-declaration", "snap-name=snapcraft"]),
            call(["snap-revision", "snap-revision=345", "snap-id=3lljuR"]),
        ]
        self.get_assertion_mock.assert_has_calls(get_assertion_calls)
        self.provider.run_mock.assert_has_calls(
            [
                call(["snap", "set", "system", "experimental.snapd-snap=true"]),
                call(["snap", "set", "system", ANY]),
                call(["snap", "watch", "--last=auto-refresh?"]),
                call(["snap", "ack", "/var/tmp/snapd.assert"]),
                call(["snap", "install", "/var/tmp/snapd.snap"]),
                call(["snap", "switch", "snapd", "--channel", "latest/edge"]),
                call(["snap", "ack", "/var/tmp/core18.assert"]),
                call(["snap", "install", "/var/tmp/core18.snap"]),
                call(["snap", "switch", "core18", "--channel", "latest/beta"]),
                call(["snap", "ack", "/var/tmp/snapcraft.assert"]),
                call(["snap", "install", "--classic", "/var/tmp/snapcraft.snap"]),
                call(["snap", "switch", "snapcraft", "--channel", "latest/candidate"]),
            ]
        )
        self.provider.push_file_mock.assert_has_calls(
            [
                call(source=ANY, destination="/var/tmp/snapd.snap"),
                call(source=ANY, destination="/var/tmp/snapd.assert"),
                call(source=ANY, destination="/var/tmp/core18.snap"),
                call(source=ANY, destination="/var/tmp/core18.assert"),
                call(source=ANY, destination="/var/tmp/snapcraft.snap"),
                call(source=ANY, destination="/var/tmp/snapcraft.assert"),
            ]
        )
        self.assertThat(
            self.registry_filepath,
            FileContains(
                dedent(
                    """\
                    core18:
                    - revision: '123'
                    snapcraft:
                    - revision: '345'
                    snapd:
                    - revision: '1'
                    """
                )
            ),
        )

    def test_snapcraft_installed_on_host_from_store_but_injection_disabled(self):
        self.useFixture(fixture_setup.FakeStore())

        snap_injector = SnapInjector(
            registry_filepath=self.registry_filepath,
            runner=self.provider._run,
            file_pusher=self.provider._push_file,
            inject_from_host=False,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")
        snap_injector.apply()

        self.get_assertion_mock.assert_not_called()
        self.provider.run_mock.assert_has_calls(
            [
                call(["snap", "set", "system", ANY]),
                call(["snap", "watch", "--last=auto-refresh?"]),
                call(["snap", "install", "--channel", "stable", "core"]),
                call(
                    ["snap", "install", "--classic", "--channel", "stable", "snapcraft"]
                ),
            ]
        )
        self.provider.push_file_mock.assert_not_called()
        self.assertThat(
            self.registry_filepath,
            FileContains(
                dedent(
                    """\
                    core:
                    - revision: '10000'
                    snapcraft:
                    - revision: '25'
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
            {
                "name": "snapcraft",
                "confinement": "classic",
                "revision": "x20",
                "tracking-channel": "latest/stable",
            },
        ]
        self.get_assertion_mock.side_effect = [
            b"fake-assertion-account-store",
            b"fake-assertion-declaration-core",
            b"fake-assertion-revision-core-123",
        ]

        snap_injector = SnapInjector(
            registry_filepath=self.registry_filepath,
            runner=self.provider._run,
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
        # Check the call count to ensure the snap switch command does not sneak in.
        self.assertThat(self.provider.run_mock.call_count, Equals(6))
        self.provider.run_mock.assert_has_calls(
            [
                call(["snap", "set", "system", ANY]),
                call(["snap", "watch", "--last=auto-refresh?"]),
                call(["snap", "ack", "/var/tmp/core.assert"]),
                call(["snap", "install", "/var/tmp/core.snap"]),
                call(["snap", "ack", "/var/tmp/snapcraft.assert"]),
                call(
                    [
                        "snap",
                        "install",
                        "--dangerous",
                        "--classic",
                        "/var/tmp/snapcraft.snap",
                    ]
                ),
            ]
        )
        self.assertThat(
            self.registry_filepath,
            FileContains(
                dedent(
                    """\
                    core:
                    - revision: '123'
                    snapcraft:
                    - revision: x20
                    """
                )
            ),
        )
        self.provider.push_file_mock.assert_has_calls(
            [
                call(source=ANY, destination="/var/tmp/core.snap"),
                call(source=ANY, destination="/var/tmp/core.assert"),
                call(source=ANY, destination="/var/tmp/snapcraft.snap"),
                call(source=ANY, destination="/var/tmp/snapcraft.assert"),
            ]
        )

    def test_snapcraft_not_installed_on_host(self):
        self.useFixture(fixture_setup.FakeStore())

        snap_injector = SnapInjector(
            registry_filepath=self.registry_filepath,
            runner=self.provider._run,
            file_pusher=self.provider._push_file,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")
        snap_injector.apply()

        self.get_assertion_mock.assert_not_called()
        self.provider.run_mock.assert_has_calls(
            [
                call(["snap", "set", "system", ANY]),
                call(["snap", "watch", "--last=auto-refresh?"]),
                call(["snap", "install", "--channel", "stable", "core"]),
                call(
                    ["snap", "install", "--classic", "--channel", "stable", "snapcraft"]
                ),
            ]
        )
        self.provider.push_file_mock.assert_not_called()
        self.assertThat(
            self.registry_filepath,
            FileContains(
                dedent(
                    """\
                    core:
                    - revision: '10000'
                    snapcraft:
                    - revision: '25'
                    """
                )
            ),
        )

    def test_snapcraft_not_installed_on_host_with_channel_from_environment(self):
        self.useFixture(fixture_setup.FakeStore())
        self.useFixture(
            fixtures.EnvironmentVariable(
                "SNAPCRAFT_BUILD_ENVIRONMENT_CHANNEL_SNAPCRAFT", "latest/edge"
            )
        )

        snap_injector = SnapInjector(
            registry_filepath=self.registry_filepath,
            runner=self.provider._run,
            file_pusher=self.provider._push_file,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")
        snap_injector.apply()

        self.get_assertion_mock.assert_not_called()
        self.provider.run_mock.assert_has_calls(
            [
                call(["snap", "set", "system", ANY]),
                call(["snap", "watch", "--last=auto-refresh?"]),
                call(["snap", "install", "--channel", "stable", "core"]),
                call(
                    ["snap", "install", "--classic", "--channel", "edge", "snapcraft"]
                ),
            ]
        )
        self.provider.push_file_mock.assert_not_called()
        self.assertThat(
            self.registry_filepath,
            FileContains(
                dedent(
                    """\
                    core:
                    - revision: '10000'
                    snapcraft:
                    - revision: '25'
                    """
                )
            ),
        )

    def test_no_registry(self):
        self.useFixture(fixture_setup.FakeStore())

        snap_injector = SnapInjector(
            registry_filepath=None,
            runner=self.provider._run,
            file_pusher=self.provider._push_file,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")
        snap_injector.apply()

        self.provider.run_mock.assert_has_calls(
            [
                call(["snap", "set", "system", ANY]),
                call(["snap", "watch", "--last=auto-refresh?"]),
                call(["snap", "install", "--channel", "stable", "core"]),
                call(
                    ["snap", "install", "--classic", "--channel", "stable", "snapcraft"]
                ),
            ]
        )
        self.provider.run_mock.reset_mock()

        snap_injector = SnapInjector(
            registry_filepath=self.registry_filepath,
            runner=self.provider._run,
            file_pusher=self.provider._push_file,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")
        snap_injector.apply()

        self.provider.run_mock.assert_has_calls(
            [
                call(["snap", "set", "system", ANY]),
                call(["snap", "watch", "--last=auto-refresh?"]),
                call(["snap", "install", "--channel", "stable", "core"]),
                call(
                    ["snap", "install", "--classic", "--channel", "stable", "snapcraft"]
                ),
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
            b"fake-assertion-account-store",
            b"fake-assertion-declaration-snapcraft",
            b"fake-assertion-revision-snapcraft-345",
        ]

        snap_injector = SnapInjector(
            registry_filepath=self.registry_filepath,
            runner=self.provider._run,
            file_pusher=self.provider._push_file,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")
        snap_injector.apply()
        self.provider.run_mock.reset_mock()

        snap_injector = SnapInjector(
            registry_filepath=self.registry_filepath,
            runner=self.provider._run,
            file_pusher=self.provider._push_file,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")
        snap_injector.apply()

        self.provider.run_mock.assert_not_called()

    def test_snapcraft_installed_on_host_from_store_rerun_refreshes(self):
        self.useFixture(fixture_setup.FakeStore())

        snap_injector = SnapInjector(
            registry_filepath=self.registry_filepath,
            runner=self.provider._run,
            file_pusher=self.provider._push_file,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")
        snap_injector.apply()
        self.provider.run_mock.reset_mock()

        snap_injector = SnapInjector(
            registry_filepath=self.registry_filepath,
            runner=self.provider._run,
            file_pusher=self.provider._push_file,
        )
        snap_injector.add("core")
        snap_injector.add("snapcraft")
        snap_injector.apply()

        self.provider.run_mock.assert_has_calls(
            [
                call(["snap", "set", "system", ANY]),
                call(["snap", "watch", "--last=auto-refresh?"]),
                call(["snap", "refresh", "--channel", "stable", "core"]),
                call(
                    ["snap", "refresh", "--classic", "--channel", "stable", "snapcraft"]
                ),
            ]
        )

    def test_snapd_not_on_host_installs_from_store(self):
        self.useFixture(fixture_setup.FakeStore())

        snap_injector = SnapInjector(
            registry_filepath=self.registry_filepath,
            runner=self.provider._run,
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
                call(["snap", "set", "system", ANY]),
                call(["snap", "watch", "--last=auto-refresh?"]),
                call(["snap", "install", "--channel", "stable", "core"]),
                call(
                    ["snap", "install", "--classic", "--channel", "stable", "snapcraft"]
                ),
            ]
        )
        self.provider.push_file_mock.assert_not_called()
        self.assertThat(
            self.registry_filepath,
            FileContains(
                dedent(
                    """\
                    core:
                    - revision: '10000'
                    snapcraft:
                    - revision: '25'
                    """
                )
            ),
        )


class GetChannelTest(unit.TestCase):
    def setUp(self):
        super().setUp()

        self.fake_logger = fixtures.FakeLogger(level=logging.WARNING)
        self.useFixture(self.fake_logger)

    def test_default_channel_for_snapcraft(self):
        self.assertThat(str(_get_snap_channel("snapcraft")), Equals("latest/stable"))
        self.assertThat(
            self.fake_logger.output,
            Not(
                Contains(
                    "SNAPCRAFT_BUILD_ENVIRONMENT_CHANNEL_SNAPCRAFT is set: installing "
                    "snapcraft from"
                )
            ),
        )

    def test_channel_for_snapcraft_from_env(self):
        self.useFixture(
            fixtures.EnvironmentVariable(
                "SNAPCRAFT_BUILD_ENVIRONMENT_CHANNEL_SNAPCRAFT", "latest/edge"
            )
        )
        self.assertThat(str(_get_snap_channel("snapcraft")), Equals("latest/edge"))
        self.assertThat(
            self.fake_logger.output,
            Contains(
                "SNAPCRAFT_BUILD_ENVIRONMENT_CHANNEL_SNAPCRAFT is set: installing "
                "snapcraft from latest/edge"
            ),
        )

    def test_default_channel_for_other_snap(self):
        self.assertThat(str(_get_snap_channel("core")), Equals("latest/stable"))
        self.assertThat(
            self.fake_logger.output,
            Not(
                Contains(
                    "SNAPCRAFT_BUILD_ENVIRONMENT_CHANNEL_SNAPCRAFT is set: installing "
                    "snapcraft from"
                )
            ),
        )

    def test_channel_for_other_snap_not_affected_by_env(self):
        self.useFixture(
            fixtures.EnvironmentVariable(
                "SNAPCRAFT_BUILD_ENVIRONMENT_CHANNEL_SNAPCRAFT", "latest/edge"
            )
        )
        self.assertThat(str(_get_snap_channel("core")), Equals("latest/stable"))
        self.assertThat(
            self.fake_logger.output,
            Not(
                Contains(
                    "SNAPCRAFT_BUILD_ENVIRONMENT_CHANNEL_SNAPCRAFT is set: installing "
                    "snapcraft from"
                )
            ),
        )
