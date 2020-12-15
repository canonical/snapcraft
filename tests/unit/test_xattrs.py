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

import os
import sys
from unittest import mock

from testtools.matchers import Equals

from snapcraft.internal import xattrs
from snapcraft.internal.errors import XAttributeTooLongError
from tests import unit


class TestXattrs(unit.TestCase):
    def setUp(self):
        super().setUp()
        self.test_file = os.path.join(self.path, "test-file")
        open(self.test_file, "w").close()

    def test_read_origin_stage_package(self):
        if sys.platform == "linux":
            result = xattrs.read_origin_stage_package(self.test_file)
            self.assertThat(result, Equals(None))
        else:
            self.assertRaises(
                RuntimeError, xattrs.read_origin_stage_package, self.test_file
            )

    def test_write_origin_stage_package(self):
        package = "foo-1.0"
        if sys.platform == "linux":
            result = xattrs.read_origin_stage_package(self.test_file)
            self.assertThat(result, Equals(None))

            xattrs.write_origin_stage_package(self.test_file, package)
            result = xattrs.read_origin_stage_package(self.test_file)
            self.assertThat(result, Equals(package))
        else:
            self.assertRaises(
                RuntimeError, xattrs.write_origin_stage_package, self.test_file, package
            )

    def test_write_origin_stage_package_long(self):
        package = "a" * 100000
        if sys.platform == "linux":
            result = xattrs.read_origin_stage_package(self.test_file)
            self.assertThat(result, Equals(None))

            self.assertRaises(
                XAttributeTooLongError,
                xattrs.write_origin_stage_package,
                self.test_file,
                package,
            )

            result = xattrs.read_origin_stage_package(self.test_file)
            self.assertThat(result, Equals(None))
        else:
            self.assertRaises(
                RuntimeError, xattrs.write_origin_stage_package, self.test_file, package
            )

    def test_symlink(self):
        test_symlink = self.test_file + "-symlink"
        os.symlink(self.test_file, test_symlink)

        if sys.platform != "linux":
            return

        result = xattrs._read_snapcraft_xattr(test_symlink, "attr")
        self.assertThat(result, Equals(None))

        xattrs._write_snapcraft_xattr(test_symlink, "attr", "value")
        result = xattrs._read_snapcraft_xattr(test_symlink, "attr")
        self.assertThat(result, Equals(None))

    @mock.patch("sys.platform", return_value="win32")
    def test_read_non_linux(self, mock_platform):
        self.assertRaises(
            RuntimeError, xattrs._read_snapcraft_xattr, self.test_file, "attr"
        )

    @mock.patch("sys.platform", return_value="win32")
    def test_write_non_linux(self, mock_platform):
        self.assertRaises(
            RuntimeError, xattrs._write_snapcraft_xattr, self.test_file, "attr", "value"
        )
