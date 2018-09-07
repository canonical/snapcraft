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

import codecs
import os
import re
import sys


# Common distribution data
name = "snapcraft"
version = "devel"
description = "Easily craft snaps from multiple sources"
author_email = "snapcraft@lists.snapcraft.io"
url = "https://github.com/snapcore/snapcraft"
packages = [
    "snapcraft",
    "snapcraft.cli",
    "snapcraft.cli.snapcraftctl",
    "snapcraft.extractors",
    "snapcraft.integrations",
    "snapcraft.internal",
    "snapcraft.internal.cache",
    "snapcraft.internal.build_providers",
    "snapcraft.internal.build_providers._multipass",
    "snapcraft.internal.deltas",
    "snapcraft.internal.lifecycle",
    "snapcraft.internal.lxd",
    "snapcraft.internal.meta",
    "snapcraft.internal.pluginhandler",
    "snapcraft.internal.project_loader",
    "snapcraft.internal.project_loader.grammar",
    "snapcraft.internal.project_loader.grammar_processing",
    "snapcraft.internal.project_loader.inspection",
    "snapcraft.internal.project_loader._extensions",
    "snapcraft.internal.repo",
    "snapcraft.internal.sources",
    "snapcraft.internal.states",
    "snapcraft.project",
    "snapcraft.plugins",
    "snapcraft.plugins._ros",
    "snapcraft.plugins._python",
    "snapcraft.storeapi",
]
package_data = {"snapcraft.internal.repo": ["manifest.txt"]}
license = "GPL v3"
classifiers = (
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
)

# look/set what version we have
changelog = "debian/changelog"
if os.path.exists(changelog):
    head = codecs.open(changelog, encoding="utf-8").readline()
    match = re.compile(".*\((.*)\).*").match(head)
    if match:
        version = match.group(1)


# If on Windows, construct an exe distribution
if sys.platform == "win32":
    from cx_Freeze import setup, Executable

    # cx_Freeze relevant options
    build_exe_options = {
        # Explicitly add any missed packages that are not found at runtime
        "packages": [
            "pkg_resources",
            "pymacaroons",
            "click",
            "responses",
            "configparser",
            "docopt",
            "cffi",
        ],
        # Explicit inclusion data, which is then clobbered.
        "include_files": [
            ("libraries", os.path.join("share", "snapcraft", "libraries")),
            ("schema", os.path.join("share", "snapcraft", "schema")),
            ("extensions", os.path.join("share", "snapcraft", "extensions")),
        ],
    }

    exe = Executable(script="bin/snapcraft", base=None)  # console subsystem

    setup(
        name=name,
        version=version,
        description=description,
        author_email=author_email,
        url=url,
        packages=packages,
        package_data=package_data,
        license=license,
        classifiers=classifiers,
        # cx_Freeze-specific arguments
        options={"build_exe": build_exe_options},
        executables=[exe],
    )

# On other platforms, continue as normal
else:
    from setuptools import setup

    setup(
        name=name,
        version=version,
        description=description,
        author_email=author_email,
        url=url,
        packages=packages,
        package_data=package_data,
        license=license,
        classifiers=classifiers,
        # non-cx_Freeze arguments
        entry_points={
            "console_scripts": [
                "snapcraft = snapcraft.cli.__main__:run",
                "snapcraft-parser = snapcraft.internal.parser:main",
            ]
        },
        # This is not in console_scripts because we need a clean environment
        scripts=["bin/snapcraftctl"],
        data_files=[
            ("share/snapcraft/schema", ["schema/" + x for x in os.listdir("schema")]),
            (
                "share/snapcraft/libraries",
                ["libraries/" + x for x in os.listdir("libraries")],
            ),
            (
                "share/snapcraft/extensions",
                ["extensions/" + x for x in os.listdir("extensions")],
            ),
        ],
        install_requires=["pysha3", "pyxdg", "requests"],
        test_suite="tests.unit",
    )
