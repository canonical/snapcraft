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

import distutils.util
import os
import sys
from typing import Optional

from snapcraft.internal.errors import XAttributeError, XAttributeTooLongError


def _get_snapcraft_xattr_key(snapcraft_key: str) -> str:
    return f"user.snapcraft.{snapcraft_key}"


def _read_snapcraft_xattr(path: str, snapcraft_key: str) -> Optional[str]:
    if sys.platform != "linux":
        raise RuntimeError("xattr support only available for Linux")

    # Extended attributes do not apply to symlinks.
    if os.path.islink(path):
        return None

    key = _get_snapcraft_xattr_key(snapcraft_key)
    try:
        value = os.getxattr(path, key)
    except OSError as error:
        # No label present with:
        # OSError: [Errno 61] No data available: b'<path>'
        if error.errno == 61:
            return None

        # Chain unknown variants of OSError.
        raise XAttributeError(action="read", key=key, path=path) from error

    return value.decode().strip()


def _write_snapcraft_xattr(path: str, snapcraft_key: str, value: str) -> None:
    if sys.platform != "linux":
        raise RuntimeError("xattr support only available for Linux")

    # Extended attributes do not apply to symlinks.
    if os.path.islink(path):
        return

    key = _get_snapcraft_xattr_key(snapcraft_key)

    try:
        os.setxattr(path, key, value.encode())
    except OSError as error:
        # Label is too long for filesystem:
        # OSError: [Errno 7] Argument list too long: b'<path>'
        if error.errno == 7:
            raise XAttributeTooLongError(path=path, key=key, value=value)

        # Chain unknown variants of OSError.
        raise XAttributeError(action="write", key=key, path=path) from error


def read_origin_stage_package(path: str) -> Optional[str]:
    """Read origin stage package."""
    try:
        return _read_snapcraft_xattr(path, "origin_stage_package")
    except XAttributeError:
        # Ignore error if origin stage package not required.
        if distutils.util.strtobool(os.environ.get("SNAPCRAFT_BUILD_INFO", "n")):
            raise
        return None


def write_origin_stage_package(path: str, value: str) -> None:
    """Write origin stage package."""
    try:
        _write_snapcraft_xattr(path, "origin_stage_package", value)
    except XAttributeError:
        # Ignore error if origin stage package not required.
        if distutils.util.strtobool(os.environ.get("SNAPCRAFT_BUILD_INFO", "n")):
            raise
