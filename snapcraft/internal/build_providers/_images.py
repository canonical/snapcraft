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
import subprocess
import tempfile
from typing import Dict

import requests

from . import errors
from snapcraft.file_utils import calculate_hash, get_tool_path
from snapcraft.internal.cache import FileCache
from snapcraft.internal.indicators import download_requests_stream


class _Image:
    def __init__(
        self, *, base: str, snap_arch: str, url: str, checksum: str, algorithm: str
    ) -> None:
        self.base = base
        self.snap_arch = snap_arch
        self.url = url
        self.checksum = checksum
        self.algorithm = algorithm

        self._image_cache = FileCache(namespace="build-images-{}".format(self.base))

    def _download_and_cache(self) -> str:
        request = requests.get(self.url, stream=True, allow_redirects=True)
        if not request.ok:
            raise errors.BuildImageRequestError(
                base=self.base, status_code=request.status_code
            )
        # First create the prefix as tempfile.TemporaryDirectory does not do that for you
        os.makedirs(self._image_cache.file_cache, exist_ok=True)
        with tempfile.TemporaryDirectory(
            prefix=self._image_cache.file_cache
        ) as tmp_dir:
            download_file = os.path.join(tmp_dir, "{}-vm".format(self.base))
            download_requests_stream(request, download_file)
            calculated_digest = calculate_hash(download_file, algorithm=self.algorithm)
            if self.checksum != calculated_digest:
                raise errors.BuildImageChecksumError(
                    expected=self.checksum,
                    calculated=calculated_digest,
                    algorithm=self.algorithm,
                )
            return self._image_cache.cache(
                filename=download_file, algorithm=self.algorithm, hash=self.checksum
            )

    def get(self):
        cached_file = self._image_cache.get(
            hash=self.checksum, algorithm=self.algorithm
        )
        if not cached_file:
            cached_file = self._download_and_cache()
        # TODO verify nothing is using this as a backing store before implementing.
        # image_cache.prune(keep_hash=image.checksum)
        return cached_file


def _get_build_images(base: str) -> Dict[str, _Image]:
    if base == "core16":
        return dict(
            amd64=_Image(
                base="core16",
                snap_arch="amd64",
                url="https://cloud-images.ubuntu.com/releases/16.04/release-20180703/ubuntu-16.04-server-cloudimg-amd64-disk1.img",  # noqa: E501
                checksum="79549e87ddfc61b1cc8626a67ccc025cd7111d1af93ec28ea46ba6de70819f8c",  # noqa: E501
                algorithm="sha256",
            )
        )
    elif base == "core18":
        return dict(
            amd64=_Image(
                base="core18",
                snap_arch="amd64",
                url="https://cloud-images.ubuntu.com/releases/18.04/release-20180724/ubuntu-18.04-server-cloudimg-amd64.img",  # noqa: E501
                checksum="6d663a8fd5eddd916f4aef4fd06d0f7f4cf0bb191f170b8c84cd2adf297bc5c3",  # noqa: E501
                algorithm="sha256",
            )
        )
    else:
        raise KeyError(base)


def setup(*, base: str, snap_arch: str, size: str, image_path: str) -> None:
    """Setup a build image for base and snap_arch of size at image_path.

    Example usage:
    >>> from snapcraft.internal.build_providers import _images
    >>> _images.setup(base="core18", snap_arch="amd64", size="10G",
                      image_path="images/core18.qcow2")

    :param str base: the base of the build image to setup.
    :param str snap_arch: the architecture of the base for the build image.
    :param str size: the size of the disk for the build image.
    :param str image_path: the path to create the build image.
    :raises errors.BuildImageForBaseMissing:
        if there is no build image defined for the requested base or snap
        architecture.
    :raises errors.BuildImageRequestError:
        upon a network related issue that prevents download of the build image.
    :raises errors.BuildImageChecksumError:
        if the resulting downloaded build image does not match the expected
        checksum.
    :raises errors.BuildImageSetupError:
        if a build image cannot be created due to tooling or other system
        issues (e.g.; space issues).
    """
    try:
        image = _get_build_images(base)[snap_arch]
    except KeyError as key_error:
        raise errors.BuildImageForBaseMissing(
            base=base, snap_arch=snap_arch
        ) from key_error

    cached_file = image.get()

    if os.path.dirname(image_path):
        os.makedirs(os.path.dirname(image_path), exist_ok=True)
    qemu_img_cmd = get_tool_path("qemu-img")
    # qemu-img parameters:
    # -q: quiet.
    # -f: first image format.
    # -b: backing file.
    try:
        subprocess.check_call(
            [
                qemu_img_cmd,
                "create",
                "-q",
                "-f",
                "qcow2",
                "-b",
                cached_file,
                image_path,
                size,
            ]
        )
    except subprocess.CalledProcessError as process_error:
        raise errors.BuildImageSetupError(
            exit_code=process_error.returncode
        ) from process_error
