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
import requests
import subprocess
from unittest import mock

from testtools.matchers import Equals, DirExists, FileExists

from snapcraft.internal.build_providers import errors, _images
from tests import unit


class ImageTest(unit.FakeFileHTTPServerBasedTestCase):
    def test_get(self):
        image = _images._Image(
            base="core18",
            snap_arch="amd64",
            url="http://{}:{}/image".format(*self.server.server_address),
            checksum="1eaacf5d02554283dca5ff3488c6a9fc6fa07e16b8282901d39245f8614d9063",
            algorithm="sha256",
        )
        image_filepath = image.get()
        self.assertThat(image_filepath, FileExists())

    def test_get_twice_uses_cached_file(self):
        image = _images._Image(
            base="core18",
            snap_arch="amd64",
            url="http://{}:{}/image".format(*self.server.server_address),
            checksum="1eaacf5d02554283dca5ff3488c6a9fc6fa07e16b8282901d39245f8614d9063",
            algorithm="sha256",
        )

        with mock.patch(
            "requests.get", new=mock.Mock(wraps=requests.get)
        ) as download_spy:
            image.get()
            image.get()
            self.assertThat(download_spy.call_count, Equals(1))

    def test_get_bad_request(self):
        image = _images._Image(
            base="core18",
            snap_arch="amd64",
            url="http://{}:{}/404-not-found".format(*self.server.server_address),
            checksum="1234567890",
            algorithm="sha256",
        )
        self.assertRaises(errors.BuildImageRequestError, image.get)

    def test_get_bad_checksum(self):
        image = _images._Image(
            base="core18",
            snap_arch="amd64",
            url="http://{}:{}/image".format(*self.server.server_address),
            checksum="1234567890",
            algorithm="sha256",
        )
        self.assertRaises(errors.BuildImageChecksumError, image.get)


class SetupTest(unit.TestCase):
    def test_setup(self):
        patcher = mock.patch.object(_images._Image, "get")
        image_get_mock = patcher.start()
        image_get_mock.return_value = "base-build-image.qcow2"
        self.addCleanup(patcher.stop)

        patcher = mock.patch("subprocess.check_call")
        call_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch.object(_images, "get_tool_path")
        tool_mock = patcher.start()
        tool_mock.return_value = "qemu-img-fake-tool"
        self.addCleanup(patcher.stop)

        _images.setup(
            base="core16", snap_arch="amd64", size="1G", image_path="image.qcow2"
        )

        image_get_mock.assert_called_once_with()
        call_mock.assert_called_once_with(
            [
                "qemu-img-fake-tool",
                "create",
                "-q",
                "-f",
                "qcow2",
                "-b",
                "base-build-image.qcow2",
                "image.qcow2",
                "1G",
            ]
        )

    def test_setup_with_dir(self):
        patcher = mock.patch.object(_images._Image, "get")
        image_get_mock = patcher.start()
        image_get_mock.return_value = "base-build-image.qcow2"
        self.addCleanup(patcher.stop)

        patcher = mock.patch("subprocess.check_call")
        call_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = mock.patch.object(_images, "get_tool_path")
        tool_mock = patcher.start()
        tool_mock.return_value = "qemu-img-fake-tool"
        self.addCleanup(patcher.stop)

        _images.setup(
            base="core16",
            snap_arch="amd64",
            size="1G",
            image_path=os.path.join("dir", "image.qcow2"),
        )

        self.assertThat("dir", DirExists())
        call_mock.assert_called_once_with(
            [
                "qemu-img-fake-tool",
                "create",
                "-q",
                "-f",
                "qcow2",
                "-b",
                "base-build-image.qcow2",
                os.path.join("dir", "image.qcow2"),
                "1G",
            ]
        )

    def test_qemu_img_call_fails(self):
        patcher = mock.patch.object(_images._Image, "get")
        image_get_mock = patcher.start()
        image_get_mock.return_value = "base-build-image.qcow2"
        self.addCleanup(patcher.stop)

        patcher = mock.patch("subprocess.check_call")
        call_mock = patcher.start()
        call_mock.side_effect = subprocess.CalledProcessError(1, ["qemu-img"])
        self.addCleanup(patcher.stop)

        patcher = mock.patch.object(_images, "get_tool_path")
        tool_mock = patcher.start()
        tool_mock.return_value = "qemu-img-fake-tool"
        self.addCleanup(patcher.stop)

        self.assertRaises(
            errors.BuildImageSetupError,
            _images.setup,
            base="core16",
            snap_arch="amd64",
            size="1G",
            image_path="image.qcow2",
        )

    def test_setup_bad_base(self):
        self.assertRaises(
            errors.BuildImageForBaseMissing,
            _images.setup,
            base="bad-base",
            snap_arch="amd64",
            size="1G",
            image_path="image.qcow2",
        )

    def test_setup_bad_snap_arch(self):
        self.assertRaises(
            errors.BuildImageForBaseMissing,
            _images.setup,
            base="core18",
            snap_arch="bad-arch",
            size="1G",
            image_path="image.qcow2",
        )
