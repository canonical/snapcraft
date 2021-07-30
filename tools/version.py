#!/usr/bin/env python3
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2021 Canonical Ltd
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

import fileinput
import pathlib
import subprocess
import sys


def determine_version():
    # Examples (git describe -> python package version):
    # 4.1.1-0-gad012482d -> 4.1.1
    # 4.1.1-16-g2d8943dbc -> 4.1.1.post16+g2d8943dbc
    #
    # For shallow clones or repositories missing tags:
    # 0ae7c04

    desc = (
        subprocess.run(
            ["git", "describe", "--always", "--long"],
            check=True,
            stdout=subprocess.PIPE,
        )
        .stdout.decode()
        .strip()
    )

    split_desc = desc.split("-")
    assert (
        len(split_desc) == 3
    ), f"Failed to parse Snapcraft git version description {desc!r}. Confirm that git repository is present and has the required tags/history."

    version = split_desc[0]
    distance = split_desc[1]
    commit = split_desc[2]

    if distance == "0":
        return version

    return f"{version}.post{distance}+git{commit[1:]}"


def set_snapcraft_iss():
    snapcraft_iss_path = pathlib.Path("windows/snapcraft.iss")
    assert (
        snapcraft_iss_path.exists()
    ), f"Run from project root and ensure {snapcraft_iss_path!s} exists."
    with fileinput.input(str(snapcraft_iss_path), inplace=True) as iss_file:
        for line in iss_file:
            if line.startswith("AppVersion="):
                print(f"AppVersion={determine_version()}")
            else:
                print(line, end="")


if __name__ == "__main__":
    if len(sys.argv) == 2 and sys.argv[1] == "set-snapcraft-iss":
        set_snapcraft_iss()
    else:
        print(determine_version())
