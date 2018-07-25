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

import logging
from textwrap import dedent

import fixtures
from testtools.matchers import Equals, HasLength

from snapcraft.internal import errors, mountinfo

from tests import unit


class MountInfoTestCase(unit.TestCase):
    def _write_mountinfo(self, contents):
        path = "mountinfo"
        with open(path, "w") as f:
            f.write(contents)
        return path

    def test_mountinfo_by_root(self):
        mounts = mountinfo.MountInfo(
            mountinfo_file=self._write_mountinfo(
                dedent(
                    """\
                23 28 0:4 / /proc rw,nosuid,nodev,noexec,relatime shared:14 - proc proc rw
                1341 28 7:6 / /snap/snapcraft/1 ro,nodev,relatime shared:39 - squashfs /dev/loop6 ro
                1455 28 253:0 /test-snap/prime /snap/test-snap/x1 ro,relatime shared:1 - ext4 /dev/mapper/foo rw,errors=remount-ro,data=ordered
                """
                )
            )
        )  # noqa

        root_mounts = mounts.for_root("/")
        for mount_point in ("/proc", "/snap/snapcraft/1"):
            self.assertTrue(
                any(m for m in root_mounts if m.mount_point == mount_point),
                "Expected {!r} to be included in root mounts".format(mount_point),
            )

        test_snap_mounts = mounts.for_root("/test-snap/prime")
        self.assertThat(test_snap_mounts, HasLength(1))
        self.expectThat(test_snap_mounts[0].mount_point, Equals("/snap/test-snap/x1"))

    def test_mountinfo_by_mount_point(self):
        mounts = mountinfo.MountInfo(
            mountinfo_file=self._write_mountinfo(
                dedent(
                    """\
                23 28 0:4 / /proc rw,nosuid,nodev,noexec,relatime shared:14 - proc proc rw
                1341 28 7:6 / /snap/snapcraft/1 ro,nodev,relatime shared:39 - squashfs /dev/loop6 ro
                1455 28 253:0 /test-snap/prime /snap/test-snap/x1 ro,relatime shared:1 - ext4 /dev/mapper/foo rw,errors=remount-ro,data=ordered
                """
                )
            )
        )  # noqa

        mount = mounts.for_mount_point("/proc")
        self.assertThat(mount.mount_id, Equals("23"))
        self.assertThat(mount.parent_id, Equals("28"))
        self.assertThat(mount.st_dev, Equals("0:4"))
        self.assertThat(mount.root, Equals("/"))
        self.assertThat(mount.mount_point, Equals("/proc"))
        self.assertThat(mount.mount_options, Equals("rw,nosuid,nodev,noexec,relatime"))
        self.assertThat(mount.optional_fields, Equals(["shared:14"]))
        self.assertThat(mount.filesystem_type, Equals("proc"))
        self.assertThat(mount.mount_source, Equals("proc"))
        self.assertThat(mount.super_options, Equals("rw"))

        mount = mounts.for_mount_point("/snap/snapcraft/1")
        self.assertThat(mount.mount_id, Equals("1341"))
        self.assertThat(mount.parent_id, Equals("28"))
        self.assertThat(mount.st_dev, Equals("7:6"))
        self.assertThat(mount.root, Equals("/"))
        self.assertThat(mount.mount_point, Equals("/snap/snapcraft/1"))
        self.assertThat(mount.mount_options, Equals("ro,nodev,relatime"))
        self.assertThat(mount.optional_fields, Equals(["shared:39"]))
        self.assertThat(mount.filesystem_type, Equals("squashfs"))
        self.assertThat(mount.mount_source, Equals("/dev/loop6"))
        self.assertThat(mount.super_options, Equals("ro"))

        mount = mounts.for_mount_point("/snap/test-snap/x1")
        self.assertThat(mount.mount_id, Equals("1455"))
        self.assertThat(mount.parent_id, Equals("28"))
        self.assertThat(mount.st_dev, Equals("253:0"))
        self.assertThat(mount.root, Equals("/test-snap/prime"))
        self.assertThat(mount.mount_point, Equals("/snap/test-snap/x1"))
        self.assertThat(mount.mount_options, Equals("ro,relatime"))
        self.assertThat(mount.optional_fields, Equals(["shared:1"]))
        self.assertThat(mount.filesystem_type, Equals("ext4"))
        self.assertThat(mount.mount_source, Equals("/dev/mapper/foo"))
        self.assertThat(
            mount.super_options, Equals("rw,errors=remount-ro,data=ordered")
        )

    def test_mountinfo_missing_root(self):
        mounts = mountinfo.MountInfo(mountinfo_file=self._write_mountinfo(""))
        raised = self.assertRaises(
            errors.RootNotMountedError, mounts.for_root, "test-root"
        )
        self.assertThat(raised.root, Equals("test-root"))

    def test_mountinfo_missing_mount_point(self):
        mounts = mountinfo.MountInfo(mountinfo_file=self._write_mountinfo(""))
        raised = self.assertRaises(
            errors.MountPointNotFoundError, mounts.for_mount_point, "test-root"
        )
        self.assertThat(raised.mount_point, Equals("test-root"))

    def test_invalid_mountinfo(self):
        self.fake_logger = fixtures.FakeLogger(level=logging.WARN)
        self.useFixture(self.fake_logger)

        mountinfo.MountInfo(mountinfo_file=self._write_mountinfo(dedent("I'm invalid")))

        # Assert that a warning was logged
        self.assertThat(
            self.fake_logger.output,
            Equals("Unable to parse mountinfo row: I'm invalid\n"),
        )
