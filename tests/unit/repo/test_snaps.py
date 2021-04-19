# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2021 Canonical Ltd
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

import fixtures
from testtools.matchers import Equals, FileContains, FileExists, Is

from snapcraft.internal.repo import errors, snaps
from tests import unit


class SnapPackageCurrentChannelTest(unit.TestCase):
    def assert_channels(self, snap, installed_snaps, expected):
        self.fake_snapd.snaps_result = installed_snaps
        snap_pkg = snaps.SnapPackage(snap)
        self.assertThat(snap_pkg.get_current_channel(), Equals(expected))

    def test_risk(self):
        self.assert_channels(
            snap="fake-snap-stable/stable",
            installed_snaps=[{"name": "fake-snap-stable", "channel": "stable"}],
            expected="latest/stable",
        )

    def test_track_risk(self):
        self.assert_channels(
            snap="fake-snap-stable/latest/stable",
            installed_snaps=[{"name": "fake-snap-stable", "channel": "stable"}],
            expected="latest/stable",
        )

    def test_track_risk_branch(self):
        self.assert_channels(
            snap="fake-snap-branch/candidate/branch",
            installed_snaps=[
                {"name": "fake-snap-branch", "channel": "candidate/branch"}
            ],
            expected="latest/candidate/branch",
        )


class SnapPackageIsInstalledTest(unit.TestCase):
    def assert_installed(self, snap, installed_snaps, expected):
        self.fake_snapd.snaps_result = installed_snaps
        snap_pkg = snaps.SnapPackage(snap)
        self.expectThat(snap_pkg.installed, Is(expected))
        self.expectThat(snaps.SnapPackage.is_snap_installed(snap), Is(expected))

    def test_default(self):
        self.assert_installed(
            snap="fake-snap-stable",
            installed_snaps=[{"name": "fake-snap-stable", "channel": "stable"}],
            expected=True,
        )

    def test_track_risk(self):

        self.assert_installed(
            snap="fake-snap-stable/latest/stable",
            installed_snaps=[{"name": "fake-snap-stable", "channel": "stable"}],
            expected=True,
        )

    def test_default_not_installed(self):
        self.assert_installed(snap="missing-snap", installed_snaps=[], expected=False),

    def test_track_risk_not_installed(self):
        self.assert_installed(
            snap="missing-snap/latest/stable", installed_snaps=[], expected=False
        )


class SnapPackageIsInStoreTest(unit.TestCase):
    def assert_in_store(self, snap, find_result, expected):
        self.fake_snapd.find_result = find_result
        snap_pkg = snaps.SnapPackage(snap)
        self.assertThat(snap_pkg.in_store, Is(expected))

    def test_default(self):
        self.assert_in_store(
            snap="fake-snap", find_result=[{"fake-snap": "dummy"}], expected=True
        )

    def test_track_risk(self):
        self.assert_in_store(
            snap="fake-snap/latest/stable",
            find_result=[{"fake-snap": "dummy"}],
            expected=True,
        )

    def test_default_not_in_store(self):
        self.assert_in_store(snap="missing-snap", find_result=[], expected=False)

    def test_track_risk_not_in_store(self):
        self.assert_in_store(
            snap="missing-snap/latest/stable", find_result=[], expected=False
        )


class SnapPackageIsClassicTest(unit.TestCase):
    def assert_classic(self, snap, find_result, expected):
        self.fake_snapd.find_result = find_result
        snap_pkg = snaps.SnapPackage(snap)
        self.assertThat(snap_pkg.is_classic(), Is(expected))

    def test_classic(self):
        self.assert_classic(
            snap="fake-snap/classic/stable",
            find_result=[
                {
                    "fake-snap": {
                        "channels": {"classic/stable": {"confinement": "classic"}}
                    }
                }
            ],
            expected=True,
        )

    def test_strict(self):
        self.assert_classic(
            snap="fake-snap/strict/stable",
            find_result=[
                {
                    "fake-snap": {
                        "channels": {"strict/stable": {"confinement": "strict"}}
                    }
                }
            ],
            expected=False,
        )

    def test_devmode(self):
        self.assert_classic(
            snap="fake-snap/devmode/stable",
            find_result=[
                {
                    "fake-snap": {
                        "channels": {"devmode/stable": {"confinement": "devmode"}}
                    }
                }
            ],
            expected=False,
        )


class SnapPackageIsValidTest(unit.TestCase):
    def assert_valid(self, snap, find_result, expected):
        self.fake_snapd.find_result = find_result
        snap_pkg = snaps.SnapPackage(snap)
        self.expectThat(snap_pkg.is_valid(), Is(expected))
        self.expectThat(snaps.SnapPackage.is_valid_snap(snap), Is(expected))

    def test_default(self):
        self.assert_valid(
            snap="fake-snap",
            find_result=[
                {
                    "fake-snap": {
                        "channels": {"latest/stable": {"confinement": "strict"}}
                    }
                }
            ],
            expected=True,
        )

    def test_track_risk(self):
        self.assert_valid(
            snap="fake-snap/strict/stable",
            find_result=[
                {
                    "fake-snap": {
                        "channels": {"strict/stable": {"confinement": "strict"}}
                    }
                }
            ],
            expected=True,
        )

    def test_invalid_track(self):
        self.assert_valid(
            snap="fake-snap/non-existent/edge",
            find_result=[
                {
                    "fake-snap": {
                        "channels": {"strict/stable": {"confinement": "strict"}}
                    }
                }
            ],
            expected=False,
        )

    def test_missing_snap(self):
        self.assert_valid(snap="missing-snap", find_result=[], expected=False)

    def test_installed(self):
        snap_pkg = snaps.SnapPackage("fake-snap/strict/stable/branch")
        snap_pkg.get_local_snap_info = lambda: {"channel": "strict/stable/branch"}

        self.assertTrue(snap_pkg.is_valid())

    def test_404(self):
        self.fake_snapd.find_code = 404
        self.assert_valid(snap="missing-snap", find_result=[], expected=False)


class SnapPackageLifecycleTest(unit.TestCase):
    def test_install_classic(self):
        self.fake_snapd.find_result = [
            {"fake-snap": {"channels": {"classic/stable": {"confinement": "classic"}}}}
        ]

        snap_pkg = snaps.SnapPackage("fake-snap/classic/stable")
        snap_pkg.install()
        self.assertThat(
            self.fake_snap_command.calls,
            Equals(
                [
                    ["snap", "whoami"],
                    [
                        "sudo",
                        "snap",
                        "install",
                        "fake-snap",
                        "--channel",
                        "classic/stable",
                        "--classic",
                    ],
                ]
            ),
        )

    def test_install_non_classic(self):
        self.fake_snapd.find_result = [
            {"fake-snap": {"channels": {"strict/stable": {"confinement": "strict"}}}}
        ]

        snap_pkg = snaps.SnapPackage("fake-snap/strict/stable")
        snap_pkg.install()
        self.assertThat(
            self.fake_snap_command.calls,
            Equals(
                [
                    ["snap", "whoami"],
                    [
                        "sudo",
                        "snap",
                        "install",
                        "fake-snap",
                        "--channel",
                        "strict/stable",
                    ],
                ]
            ),
        )

    def test_install_classic_not_on_channel(self):
        self.fake_snapd.find_result = [{"fake-snap": {"channels": {}}}]

        snap_pkg = snaps.SnapPackage("fake-snap/classic/stable")
        snap_pkg.install()
        self.assertThat(
            self.fake_snap_command.calls,
            Equals(
                [
                    ["snap", "whoami"],
                    [
                        "sudo",
                        "snap",
                        "install",
                        "fake-snap",
                        "--channel",
                        "classic/stable",
                    ],
                ]
            ),
        )

    def test_install_branch(self):
        snap_pkg = snaps.SnapPackage("fake-snap/strict/stable/branch")
        snap_pkg.install()
        self.assertThat(
            self.fake_snap_command.calls,
            Equals(
                [
                    ["snap", "whoami"],
                    [
                        "sudo",
                        "snap",
                        "install",
                        "fake-snap",
                        "--channel",
                        "strict/stable/branch",
                    ],
                ]
            ),
        )

    def test_download_from_host(self):
        fake_get_assertion = fixtures.MockPatch(
            "snapcraft.internal.repo.snaps.get_assertion", return_value=b"foo-assert"
        )
        self.useFixture(fake_get_assertion)

        self.fake_snapd.snaps_result = [
            {
                "id": "fake-snap-id",
                "name": "fake-snap",
                "channel": "stable",
                "revision": "10",
            }
        ]

        snap_pkg = snaps.SnapPackage("fake-snap/strict/stable")
        snap_pkg.local_download(
            snap_path="fake-snap.snap", assertion_path="fake-snap.assert"
        )

        self.assertThat("fake-snap.snap", FileExists())
        self.assertThat(
            "fake-snap.assert", FileContains("foo-assert\nfoo-assert\nfoo-assert\n")
        )
        fake_get_assertion.mock.assert_has_calls(
            [
                mock.call(
                    [
                        "account-key",
                        "public-key-sha3-384=BWDEoaqyr25nF5SNCvEv2v7QnM9QsfCc0PBMYD_i2NGSQ32EF2d4D0hqUel3m8ul",
                    ]
                ),
                mock.call(["snap-declaration", "snap-name=fake-snap"]),
                mock.call(
                    ["snap-revision", "snap-revision=10", "snap-id=fake-snap-id"]
                ),
            ]
        )

    def test_download_from_host_dangerous(self):
        fake_get_assertion = fixtures.MockPatch(
            "snapcraft.internal.repo.snaps.get_assertion", return_value=b"foo-assert"
        )
        self.useFixture(fake_get_assertion)
        self.fake_snapd.snaps_result = [
            {
                "id": "fake-snap-id",
                "name": "fake-snap",
                "channel": "stable",
                "revision": "x1",
            }
        ]

        snap_pkg = snaps.SnapPackage("fake-snap/strict/stable")
        snap_pkg.local_download(
            snap_path="fake-snap.snap", assertion_path="fake-snap.assert"
        )

        self.assertThat("fake-snap.snap", FileExists())
        self.assertThat("fake-snap.assert", FileContains(""))
        fake_get_assertion.mock.assert_not_called()

    def test_install_logged_in(self):
        self.fake_snapd.find_result = [
            {"fake-snap": {"channels": {"strict/stable": {"confinement": "strict"}}}}
        ]

        self.fake_snap_command.login("user@email.com")
        snap_pkg = snaps.SnapPackage("fake-snap/strict/stable")
        snap_pkg.install()
        self.assertThat(
            self.fake_snap_command.calls,
            Equals(
                [
                    ["snap", "whoami"],
                    ["snap", "install", "fake-snap", "--channel", "strict/stable"],
                ]
            ),
        )

    def test_install_fails(self):
        self.fake_snap_command.install_success = False
        snap_pkg = snaps.SnapPackage("fake-snap/strict/stable")
        self.assertRaises(errors.SnapInstallError, snap_pkg.install)

    def test_refresh(self):
        self.fake_snapd.find_result = [
            {"fake-snap": {"channels": {"strict/stable": {"confinement": "strict"}}}}
        ]

        snap_pkg = snaps.SnapPackage("fake-snap/strict/stable")
        snap_pkg.refresh()
        self.assertThat(
            self.fake_snap_command.calls,
            Equals(
                [
                    ["snap", "whoami"],
                    [
                        "sudo",
                        "snap",
                        "refresh",
                        "fake-snap",
                        "--channel",
                        "strict/stable",
                    ],
                ]
            ),
        )

    def test_refresh_branch(self):
        snap_pkg = snaps.SnapPackage("fake-snap/strict/stable/branch")
        snap_pkg.refresh()
        self.assertThat(
            self.fake_snap_command.calls,
            Equals(
                [
                    ["snap", "whoami"],
                    [
                        "sudo",
                        "snap",
                        "refresh",
                        "fake-snap",
                        "--channel",
                        "strict/stable/branch",
                    ],
                ]
            ),
        )

    def test_download(self):
        self.fake_snapd.find_result = [
            {"fake-snap": {"channels": {"strict/stable": {"confinement": "strict"}}}}
        ]

        snap_pkg = snaps.SnapPackage("fake-snap")
        snap_pkg.download()
        self.assertThat(
            self.fake_snap_command.calls, Equals([["snap", "download", "fake-snap"]])
        )

    def test_download_channel(self):
        self.fake_snapd.find_result = [
            {"fake-snap": {"channels": {"strict/edge": {"confinement": "strict"}}}}
        ]

        snap_pkg = snaps.SnapPackage("fake-snap/strict/stable")
        snap_pkg.download()
        self.assertThat(
            self.fake_snap_command.calls,
            Equals([["snap", "download", "fake-snap", "--channel", "strict/stable"]]),
        )

    def test_download_classic(self):
        self.fake_snapd.find_result = [
            {"fake-snap": {"channels": {"classic/stable": {"confinement": "classic"}}}}
        ]

        snap_pkg = snaps.SnapPackage("fake-snap")
        snap_pkg.download()
        self.assertThat(
            self.fake_snap_command.calls, Equals([["snap", "download", "fake-snap"]])
        )

    def test_download_snaps(self):
        self.fake_snapd.find_result = [
            {"fake-snap": {"channels": {"latest/stable": {"confinement": "strict"}}}},
            {
                "other-fake-snap": {
                    "channels": {"latest/stable": {"confinement": "strict"}}
                }
            },
        ]

        snaps.download_snaps(
            snaps_list=["fake-snap", "other-fake-snap/latest/stable"],
            directory="fakedir",
        )
        self.assertThat(
            self.fake_snap_command.calls,
            Equals(
                [
                    ["snap", "download", "fake-snap"],
                    [
                        "snap",
                        "download",
                        "other-fake-snap",
                        "--channel",
                        "latest/stable",
                    ],
                ]
            ),
        )

    def test_download_snaps_with_invalid(self):
        self.fake_snapd.find_result = [
            {"fake-snap": {"channels": {"latest/stable": {"confinement": "strict"}}}},
        ]
        self.fake_snap_command.download_side_effect = [True, False]

        self.assertRaises(
            errors.SnapDownloadError,
            snaps.download_snaps,
            snaps_list=["fake-snap", "other-invalid"],
            directory="fakedir",
        )
        self.assertThat(
            self.fake_snap_command.calls,
            Equals(
                [
                    ["snap", "download", "fake-snap"],
                    ["snap", "download", "other-invalid"],
                ]
            ),
        )

    def test_refresh_to_classic(self):
        self.fake_snapd.find_result = [
            {"fake-snap": {"channels": {"classic/stable": {"confinement": "classic"}}}}
        ]

        snap_pkg = snaps.SnapPackage("fake-snap/classic/stable")
        snap_pkg.refresh()
        self.assertThat(
            self.fake_snap_command.calls,
            Equals(
                [
                    ["snap", "whoami"],
                    [
                        "sudo",
                        "snap",
                        "refresh",
                        "fake-snap",
                        "--channel",
                        "classic/stable",
                        "--classic",
                    ],
                ]
            ),
        )

    def test_refresh_not_on_channel(self):
        self.fake_snapd.find_result = [{"fake-snap": {"channels": {}}}]

        snap_pkg = snaps.SnapPackage("fake-snap/classic/stable")
        snap_pkg.refresh()
        self.assertThat(
            self.fake_snap_command.calls,
            Equals(
                [
                    ["snap", "whoami"],
                    [
                        "sudo",
                        "snap",
                        "refresh",
                        "fake-snap",
                        "--channel",
                        "classic/stable",
                    ],
                ]
            ),
        )

    def test_refresh_logged_in(self):
        self.fake_snapd.find_result = [
            {"fake-snap": {"channels": {"strict/stable": {"confinement": "strict"}}}}
        ]

        self.fake_snap_command.login("user@email.com")
        snap_pkg = snaps.SnapPackage("fake-snap/strict/stable")
        snap_pkg.refresh()
        self.assertThat(
            self.fake_snap_command.calls,
            Equals(
                [
                    ["snap", "whoami"],
                    ["snap", "refresh", "fake-snap", "--channel", "strict/stable"],
                ]
            ),
        )

    def test_refresh_fails(self):
        snap_pkg = snaps.SnapPackage("fake-snap/strict/stable")
        self.fake_snap_command.refresh_success = False
        self.assertRaises(errors.SnapRefreshError, snap_pkg.refresh)

    def test_install_snaps_returns_revision(self):
        self.fake_snapd.find_result = [
            {
                "fake-snap": {
                    "channel": "stable",
                    "type": "app",
                    "channels": {"latest/stable": {"confinement": "strict"}},
                }
            }
        ]
        self.fake_snapd.snaps_result = [
            {
                "name": "fake-snap",
                "channel": "stable",
                "revision": "test-fake-snap-revision",
            }
        ]

        installed_snaps = snaps.install_snaps(["fake-snap"])
        self.assertThat(installed_snaps, Equals(["fake-snap=test-fake-snap-revision"]))

    def test_install_snaps_non_stable_base(self):
        self.fake_snapd.find_result = [
            {
                "fake-base-snap": {
                    "channel": "beta",
                    "type": "base",
                    "channels": {"latest/beta": {"confinement": "strict"}},
                }
            }
        ]
        self.fake_snapd.snaps_result = [
            {
                "name": "fake-base-snap",
                "channel": "beta",
                "revision": "test-fake-base-snap-revision",
            }
        ]

        installed_snaps = snaps.install_snaps(["fake-base-snap"])
        self.assertThat(
            installed_snaps, Equals(["fake-base-snap=test-fake-base-snap-revision"])
        )


class InstalledSnapsTestCase(unit.TestCase):
    def test_get_installed_snaps(self):
        self.fake_snapd.snaps_result = [
            {"name": "test-snap-1", "revision": "test-snap-1-revision"},
            {"name": "test-snap-2", "revision": "test-snap-2-revision"},
        ]
        installed_snaps = snaps.get_installed_snaps()
        self.assertThat(
            installed_snaps,
            Equals(
                ["test-snap-1=test-snap-1-revision", "test-snap-2=test-snap-2-revision"]
            ),
        )


class SnapdNotInstalledTestCase(unit.TestCase):
    def setUp(self):
        super().setUp()
        socket_path_patcher = mock.patch(
            "snapcraft.internal.repo.snaps.get_snapd_socket_path_template"
        )
        mock_socket_path = socket_path_patcher.start()
        mock_socket_path.return_value = "http+unix://nonexisting"
        self.addCleanup(socket_path_patcher.stop)

    def test_get_installed_snaps(self):
        installed_snaps = snaps.get_installed_snaps()
        self.assertThat(installed_snaps, Equals([]))
