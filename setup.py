#!/usr/bin/env python3
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2015-2018 Canonical Ltd
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
import sys

from setuptools import find_namespace_packages, setup


def recursive_data_files(directory, install_directory):
    data_files = []
    for root, directories, file_names in os.walk(directory):
        file_paths = [os.path.join(root, file_name) for file_name in file_names]
        data_files.append((os.path.join(install_directory, root), file_paths))
    return data_files


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


# Common distribution data
name = "snapcraft"
description = "Publish your app for Linux users for desktop, cloud, and IoT."
author_email = "snapcraft@lists.snapcraft.io"
url = "https://github.com/snapcore/snapcraft"
license = "GPL v3"
classifiers = [
    "Development Status :: 4 - Beta",
    "Environment :: Console",
    "Intended Audience :: Developers",
    "Intended Audience :: System Administrators",
    "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
    "Natural Language :: English",
    "Programming Language :: Python",
    "Programming Language :: Python :: 3",
    "Programming Language :: Python :: 3.5",
    "Topic :: Software Development :: Build Tools",
    "Topic :: System :: Software Distribution",
]

# snapcraftctl is not in console_scripts because we need a clean environment.
# Only include it for Linux.
if sys.platform == "linux":
    scripts = ["bin/snapcraftctl"]
else:
    scripts = []

setup(
    name=name,
    version=determine_version(),
    description=description,
    author_email=author_email,
    url=url,
    packages=find_namespace_packages(),
    license=license,
    classifiers=classifiers,
    scripts=scripts,
    entry_points=dict(console_scripts=["snapcraft = snapcraft.cli.__main__:run"]),
    data_files=(
        recursive_data_files("schema", "share/snapcraft")
        + recursive_data_files("keyrings", "share/snapcraft")
        + recursive_data_files("extensions", "share/snapcraft")
    ),
    install_requires=["pysha3", "pyxdg", "requests"],
    test_suite="tests.unit",
)
