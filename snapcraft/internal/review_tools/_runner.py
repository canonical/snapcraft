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

from . import errors


_REVIEW_TOOLS_PATH = pathlib.Path("/snap/bin/review-tools.snap-review")


def is_available() -> bool:
    return _REVIEW_TOOLS_PATH.exists()


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
