# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017-2018 Canonical Ltd
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

from testtools.matchers import Equals, Is

from snapcraft.internal.repo import errors, snaps
from tests import unit


class SnapPackageCurrentChannelTest(unit.TestCase):

    scenarios = [
        (
            "stable",
            dict(
                snap="fake-snap-stable/stable",
                installed_snaps=[{"name": "fake-snap-stable", "channel": "stable"}],
                expected="latest/stable",
            ),
        ),
        (
            "latest/stable",
            dict(
                snap="fake-snap-stable/latest/stable",
                installed_snaps=[{"name": "fake-snap-stable", "channel": "stable"}],
                expected="latest/stable",
            ),
        ),
        (
            "candidate/branch",
            dict(
                snap="fake-snap-branch/candidate/branch",
                installed_snaps=[
                    {"name": "fake-snap-branch", "channel": "candidate/branch"}
                ],
                expected="latest/candidate/branch",
            ),
        ),
        (
            "track/stable/branch",
            dict(
                snap="fake-snap-track-stable-branch/track/stable/branch",
                installed_snaps=[
                    {
                        "name": "fake-snap-track-stable-branch",
                        "channel": "track/stable/branch",
                    }
                ],
                expected="track/stable/branch",
            ),
        ),
        (
            "edge",
            dict(
                snap="fake-snap-edge/stable",
                installed_snaps=[{"name": "fake-snap-edge", "channel": "edge"}],
                expected="latest/edge",
            ),
        ),
        (
            "track/stable",
            dict(
                snap="fake-snap-track-stable/track/stable",
                installed_snaps=[
                    {"name": "fake-snap-track-stable", "channel": "track/stable"}
                ],
                expected="track/stable",
            ),
        ),
    ]

    def test_get_current_channel(self):
        self.fake_snapd.snaps_result = self.installed_snaps
        snap_pkg = snaps.SnapPackage(self.snap)
        self.assertThat(snap_pkg.get_current_channel(), Equals(self.expected))


class SnapPackageIsInstalledTest(unit.TestCase):

    scenarios = [
        (
            "installed stable",
            dict(
                snap="fake-snap-stable",
                installed_snaps=[{"name": "fake-snap-stable", "channel": "stable"}],
                expected=True,
            ),
        ),
        (
            "installed stable with channel",
            dict(
                snap="fake-snap-stable/latest/stable",
                installed_snaps=[{"name": "fake-snap-stable", "channel": "stable"}],
                expected=True,
            ),
        ),
        (
            "not installed",
            dict(snap="missing-snap", installed_snaps=[], expected=False),
        ),
        (
            "not installed with channel",
            dict(snap="missing-snap/latest/stable", installed_snaps=[], expected=False),
        ),
    ]

    def test_is_installed(self):
        self.fake_snapd.snaps_result = self.installed_snaps
        snap_pkg = snaps.SnapPackage(self.snap)
        self.assertThat(snap_pkg.installed, Is(self.expected))

    def test_is_installed_classmethod(self):
        self.fake_snapd.snaps_result = self.installed_snaps
        self.assertThat(
            snaps.SnapPackage.is_snap_installed(self.snap), Is(self.expected)
        )


class SnapPackageIsInStoreTest(unit.TestCase):

    scenarios = [
        (
            "in store",
            dict(snap="fake-snap", find_result=[{"fake-snap": "dummy"}], expected=True),
        ),
        (
            "in store with channel",
            dict(
                snap="fake-snap/latest/stable",
                find_result=[{"fake-snap": "dummy"}],
                expected=True,
            ),
        ),
        ("not in store", dict(snap="missing-snap", find_result=[], expected=False)),
        (
            "not in store with channel",
            dict(snap="missing-snap/latest/stable", find_result=[], expected=False),
        ),
    ]

    def test_is_in_store(self):
        self.fake_snapd.find_result = self.find_result
        snap_pkg = snaps.SnapPackage(self.snap)
        self.assertThat(snap_pkg.in_store, Is(self.expected))


class SnapPackageIsClassicTest(unit.TestCase):

    scenarios = [
        (
            "classic",
            dict(
                snap="fake-snap/classic/stable",
                find_result=[
                    {
                        "fake-snap": {
                            "channels": {"classic/stable": {"confinement": "classic"}}
                        }
                    }
                ],
                expected=True,
            ),
        ),
        (
            "strict",
            dict(
                snap="fake-snap/strict/stable",
                find_result=[
                    {
                        "fake-snap": {
                            "channels": {"strict/stable": {"confinement": "strict"}}
                        }
                    }
                ],
                expected=False,
            ),
        ),
        (
            "devmode",
            dict(
                snap="fake-snap/devmode/stable",
                find_result=[
                    {
                        "fake-snap": {
                            "channels": {"devmode/stable": {"confinement": "devmode"}}
                        }
                    }
                ],
                expected=False,
            ),
        ),
    ]

    def test_is_classic(self):
        self.fake_snapd.find_result = self.find_result
        snap_pkg = snaps.SnapPackage(self.snap)
        self.assertThat(snap_pkg.is_classic(), Is(self.expected))


class SnapPackageIsValidTest(unit.TestCase):

    scenarios = [
        (
            "valid",
            dict(
                snap="fake-snap",
                find_result=[
                    {
                        "fake-snap": {
                            "channels": {"latest/stable": {"confinement": "strict"}}
                        }
                    }
                ],
                expected=True,
            ),
        ),
        (
            "valid with channel",
            dict(
                snap="fake-snap/strict/stable",
                find_result=[
                    {
                        "fake-snap": {
                            "channels": {"strict/stable": {"confinement": "strict"}}
                        }
                    }
                ],
                expected=True,
            ),
        ),
        (
            "valid but invalid channel",
            dict(
                snap="fake-snap/non-existent/edge",
                find_result=[
                    {
                        "fake-snap": {
                            "channels": {"strict/stable": {"confinement": "strict"}}
                        }
                    }
                ],
                expected=False,
            ),
        ),
        ("invalid", dict(snap="missing-snap", find_result=[], expected=False)),
        (
            "invalid with channel",
            dict(snap="missing-snap/strict/stable", find_result=[], expected=False),
        ),
    ]

    def test_is_valid(self):
        self.fake_snapd.find_result = self.find_result
        snap_pkg = snaps.SnapPackage(self.snap)
        self.assertThat(snap_pkg.is_valid(), Is(self.expected))

    def test_is_valid_classmethod(self):
        self.fake_snapd.find_result = self.find_result
        self.assertThat(snaps.SnapPackage.is_valid_snap(self.snap), Is(self.expected))


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
        self.fake_snapd.find_result = [
            {"fake-snap": {"channels": {"strict/stable": {"confinement": "strict"}}}}
        ]

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
        self.fake_snapd.find_result = [
            {"fake-snap": {"channels": {"strict/stable": {"confinement": "strict"}}}}
        ]

        snap_pkg = snaps.SnapPackage("fake-snap/strict/stable")
        self.fake_snap_command.refresh_success = False
        self.assertRaises(errors.SnapRefreshError, snap_pkg.refresh)

    def test_install_snaps_returns_revision(self):
        self.fake_snapd.find_result = [
            {"fake-snap": {"channels": {"latest/stable": {"confinement": "strict"}}}}
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

    def test_install_snaps_non_existent_snap(self):
        self.fake_snapd.find_code = 404
        self.assertRaises(
            errors.SnapUnavailableError, snaps.install_snaps, ["fake-snap"]
        )

    def test_install_snaps_non_existent_channel(self):
        self.fake_snapd.find_result = [
            {"fake-snap": {"channels": {"classic/stable": {"confinement": "classic"}}}}
        ]

        self.assertRaises(
            errors.SnapUnavailableError,
            snaps.install_snaps,
            ["fake-snap/non-existent/edge"],
        )

    def test_install_multiple_snaps(self):
        self.fake_snapd.find_result = [
            {"fake-snap": {"channels": {"classic/stable": {"confinement": "classic"}}}}
        ]

        def snap_details(handler_instance, snap_name):
            if snap_name == "fake-snap":
                return (200, {"channel": "stable", "revision": "dummy"})
            # XXX The query for the new-fake-snap details must fail the first
            # time, but succeed the second.
            elif snap_name == "new-fake-snap":
                if not handler_instance._private_data["new_fake_snap_installed"]:
                    handler_instance._private_data["new_fake_snap_installed"] = True
                    return (404, {})
                else:
                    return (200, {"channel": "stable", "revision": "dummy"})

        self.fake_snapd.snap_details_func = snap_details
        snaps.install_snaps(["fake-snap/classic/stable", "new-fake-snap"])
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
                    ["snap", "whoami"],
                    ["sudo", "snap", "install", "new-fake-snap"],
                ]
            ),
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
