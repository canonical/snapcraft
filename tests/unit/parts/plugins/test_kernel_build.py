# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2026 Canonical Ltd.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License version 3 as published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import os
import subprocess
from pathlib import Path

import pytest

KERNEL_BUILD_SCRIPT = (
    Path(__file__).parents[4] / "snapcraft/parts/plugins/kernel_build.sh"
)


@pytest.mark.parametrize(
    ("version", "expected"),
    [
        ("1.2.3.4-5", "1.2.3-4"),
        ("1.2.3-4-5", "1.2.3-4"),
        ("1.2.3.4.5", "1.2.3-4"),
        ("1.2.3.4-arbitrary-suffix", "1.2.3-4"),
        ("1.2.3.4-5~24.04.1", "1.2.3-4"),
    ],
)
def test_normalize_kernel_version(version, expected):
    env = os.environ.copy()
    env["SNAPCRAFT_TESTING"] = "1"

    process = subprocess.run(
        [
            "sh",
            "-c",
            '. "$1"; normalize_kernel_version "$2"',
            "sh",
            str(KERNEL_BUILD_SCRIPT),
            version,
        ],
        check=True,
        capture_output=True,
        text=True,
        env=env,
    )

    assert process.stdout == f"{expected}\n"
