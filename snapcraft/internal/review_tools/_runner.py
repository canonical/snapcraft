# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2019 Canonical Ltd
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

import json
import pathlib
import subprocess

from snapcraft import file_utils

from . import errors

_REVIEW_TOOLS_PATH = pathlib.Path("/snap/bin/review-tools.snap-review")
_REVIEW_TOOLS_SNAP_USER_COMMON = pathlib.Path("~/snap/review-tools/common").expanduser()


def is_available() -> bool:
    return _REVIEW_TOOLS_PATH.exists()


def _get_review_tools_user_common() -> pathlib.Path:
    return _REVIEW_TOOLS_SNAP_USER_COMMON


def _snap_in_review_tools_common(method):
    def common_decorator(*, snap_filename: str) -> None:
        snap_filename_common_path = (
            _get_review_tools_user_common() / pathlib.Path(snap_filename).name
        )
        # Create the directory tree for the cases where the review-tools have not run before.
        snap_filename_common_path.parent.mkdir(parents=True, exist_ok=True)
        file_utils.link_or_copy(snap_filename, str(snap_filename_common_path))

        try:
            method(snap_filename=snap_filename_common_path)
        finally:
            if snap_filename_common_path.exists():
                snap_filename_common_path.unlink()

    return common_decorator


@_snap_in_review_tools_common
def run(*, snap_filename: str) -> None:
    # TODO allow suppression of error and warn through project configuration.
    command = [
        _REVIEW_TOOLS_PATH.as_posix(),
        snap_filename,
        "--json",
        "--allow-classic",
    ]
    # Speed up the process by not doing the re-squash tests.
    env = dict(SNAP_ENFORCE_RESQUASHFS="0")

    try:
        subprocess.check_output(command, stderr=subprocess.STDOUT, env=env)
    except FileNotFoundError:
        raise errors.ReviewToolMissing()
    except subprocess.CalledProcessError as call_error:
        # Error codes:
        # 2 -> warnings.
        # 3 -> warnings and errors.
        # Any of these trigger a store review.
        if call_error.returncode not in (2, 3):
            raise
        # We only really care about v2_lint now.
        raise errors.ReviewError(review_json=json.loads(call_error.output.decode()))
