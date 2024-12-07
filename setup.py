#!/usr/bin/env python3
# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2015-2022,2024 Canonical Ltd.
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
url = "https://github.com/canonical/snapcraft"
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
    "Programming Language :: Python :: 3.10",
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
    "codespell[toml]",
    "coverage[toml]",
    "pyflakes",
    "fixtures",
    # For Tiobe TiCS
    "flake8",
    "mccabe",
    "mypy",
    "testscenarios",
    "pexpect",
    "pip",
    "pycodestyle",
    "pydocstyle",
    "pyftpdlib",
    "pyinstaller; sys_platform == 'win32'",
    # For Tiobe TiCS
    "pylint",
    "pyramid",
    "pytest",
    "pytest-cov",
    "pytest-check",
    "pytest-mock",
    "pytest-subprocess",
    "tox>=4.5",
    "types-PyYAML",
    # types-requests>=2.31.0.7 requires urllib3>=2
    "types-requests==2.31.0.6",
    "types-setuptools",
    "types-simplejson",
    "types-tabulate",
    "types-toml",
    "yamllint",
]

install_requires = [
    "attrs",
    "catkin-pkg; sys_platform == 'linux'",
    "click",
    "craft-application~=4.4",
    "craft-archives~=2.0",
    "craft-cli~=2.9",
    "craft-grammar>=2.0.1,<3.0.0",
    "craft-parts>=2.1.2,<3.0.0",
    "craft-platforms~=0.4",
    "craft-providers>=2.0.4,<3.0.0",
    "craft-store>=3.0.2,<4.0.0",
    "docutils<0.20",  # Frozen until we can update sphinx dependencies.
    "gnupg",
    "jsonschema==2.6.0",
    "launchpadlib",
    "lazr.restfulclient",
    "lxml",
    "macaroonbakery",
    "mypy-extensions",
    "overrides",
    "packaging",
    "progressbar",
    "pydantic~=2.8",
    "pyelftools",
    # Pygit2 and libgit2 need to match versions.
    # Further info: https://www.pygit2.org/install.html#version-numbers
    "pygit2~=1.13.0",
    "pylxd; sys_platform == 'linux'",
    "pymacaroons",
    "python-apt @ https://launchpad.net/ubuntu/+archive/primary/+sourcefiles/python-apt/2.4.0ubuntu1/python-apt_2.4.0ubuntu1.tar.xz ; sys_platform == 'linux'",
    "python-debian; sys_platform == 'linux'",
    "pyxdg",
    "pyyaml",
    "raven",
    "requests-toolbelt",
    "requests-unixsocket2",
    "requests",
    "simplejson",
    "snap-helpers",
    "tabulate",
    "toml",
    "tinydb",
    "typing-extensions",
    "validators>=0.28.3",
]

docs_requires = {
    "canonical-sphinx[full]>=0.2.0",
    "sphinx-autobuild",
    "sphinx-autodoc-typehints",
    "sphinxcontrib-details-directive",
    "sphinx-lint",
    "sphinx-toolbox",
    "pyspelling",
}

extras_requires = {"dev": dev_requires, "docs": docs_requires}

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
            "snapcraft = snapcraft.application:main",
        ]
    ),
    data_files=(
        recursive_data_files("schema", "share/snapcraft")
        + recursive_data_files("keyrings", "share/snapcraft")
        + recursive_data_files("extensions", "share/snapcraft")
    ),
    include_package_data=True,
    package_data={
        "snapcraft": ["templates/*"],
    },
    python_requires=">=3.10",
    install_requires=install_requires,
    extras_require=extras_requires,
    test_suite="tests.unit",
)
