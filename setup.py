#!/usr/bin/env python3
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2015-2022 Canonical Ltd.
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
import re
import sys

from setuptools import find_namespace_packages, setup

from tools.version import determine_version


def recursive_data_files(directory, install_directory):
    data_files = []
    for root, _directories, file_names in os.walk(directory):
        file_paths = [os.path.join(root, file_name) for file_name in file_names]
        data_files.append((os.path.join(install_directory, root), file_paths))
    return data_files


# Common distribution data
name = "snapcraft"
description = "Publish your app for Linux users for desktop, cloud, and IoT."
author_email = "snapcraft@lists.snapcraft.io"
url = "https://github.com/snapcore/snapcraft"
license_ = "GPL v3"
classifiers = [
    "Development Status :: 4 - Beta",
    "Environment :: Console",
    "Intended Audience :: Developers",
    "Intended Audience :: System Administrators",
    "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
    "Natural Language :: English",
    "Programming Language :: Python",
    "Programming Language :: Python :: 3",
    "Programming Language :: Python :: 3.8",
    "Topic :: Software Development :: Build Tools",
    "Topic :: System :: Software Distribution",
]

# snapcraftctl is not in console_scripts because we need a clean environment.
# Only include it for Linux.
if sys.platform == "linux":
    scripts = ["bin/snapcraftctl", "bin/snapcraftctl-compat"]
else:
    scripts = []

dev_requires = [
    "black",
    "codespell",
    "coverage[toml]",
    "pyflakes",
    "fixtures",
    "isort",
    "mccabe",
    "mypy",
    "testscenarios",
    "pexpect",
    "pip",
    "pycodestyle",
    "pydocstyle",
    "pyftpdlib",
    "pylint",
    "pylint-fixme-info",
    "pylint-pytest",
    "pyramid",
    "pytest",
    "pytest-cov",
    "pytest-mock",
    "pytest-subprocess",
    "ruff==0.0.220",
    "tox>=4.0",
    "types-PyYAML",
    "types-requests",
    "types-setuptools",
    "types-tabulate",
]

if sys.platform == "win32":
    dev_requires.append("pyinstaller")

install_requires = [
    "attrs",
    "click",
    "craft-archives==0.0.3",
    "craft-cli",
    "craft-grammar",
    "craft-parts",
    "craft-providers",
    "craft-store",
    "docutils<0.20",  # Frozen until we can update sphinx dependencies.
    "cryptography==3.4",
    "gnupg",
    "jsonschema==2.5.1",
    "launchpadlib",
    "lazr.restfulclient",
    "lxml",
    "macaroonbakery",
    "mypy-extensions",
    "overrides",
    "progressbar",
    "pyelftools",
    "pymacaroons",
    "pyxdg",
    "pyyaml",
    "raven",
    "requests-toolbelt",
    "requests-unixsocket",
    "requests",
    # pin setuptools<66 (CRAFT-1598)
    "setuptools<66",
    "simplejson",
    "snap-helpers",
    "tabulate",
    "toml",
    "tinydb",
    "typing-extensions",
    "urllib3<2",  # requests-unixsocket does not yet work with urllib3 v2.0+
]

try:
    ubuntu = bool(
        re.search(
            r"^ID(?:_LIKE)?=.*\bubuntu\b.*$",
            open("/etc/os-release").read(),
            re.MULTILINE,
        )
    )
except FileNotFoundError:
    ubuntu = False

if sys.platform == "linux":
    install_requires += [
        "pylxd",
    ]

if ubuntu:
    install_requires += [
        "catkin-pkg",
        "python-apt",
        "python-debian",
    ]

extras_requires = {
    "dev": dev_requires,
}

setup(
    name=name,
    version=determine_version(),
    description=description,
    author_email=author_email,
    url=url,
    packages=find_namespace_packages(),
    license=license_,
    classifiers=classifiers,
    scripts=scripts,
    entry_points=dict(
        console_scripts=[
            "snapcraft_legacy = snapcraft_legacy.cli.__main__:run",
            "snapcraft = snapcraft.cli:run",
        ]
    ),
    data_files=(
        recursive_data_files("schema", "share/snapcraft")
        + recursive_data_files("keyrings", "share/snapcraft")
        + recursive_data_files("extensions", "share/snapcraft")
    ),
    install_requires=install_requires,
    extras_require=extras_requires,
    test_suite="tests.unit",
)
