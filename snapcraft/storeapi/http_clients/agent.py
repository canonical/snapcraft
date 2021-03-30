# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017,2020-2021 Canonical Ltd
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

import snapcraft
from snapcraft import project
from snapcraft.internal import os_release
from snapcraft.internal.errors import OsReleaseNameError, OsReleaseVersionIdError


def _is_ci_env():
    env_prefixes = ["TRAVIS", "AUTOPKGTEST_TMP"]
    matches = []

    for prefix in env_prefixes:
        matches += [var for var in os.environ.keys() if var.startswith(prefix)]

    return len(matches) > 0


def _get_linux_release(release: os_release.OsRelease) -> str:
    try:
        os_name = release.name()
    except OsReleaseNameError:
        os_name = "Unknown"
    try:
        os_version_id = release.version_id()
    except OsReleaseVersionIdError:
        os_version_id = "Unknown Version"

    return f"{os_name}/{os_version_id}"


def get_user_agent(platform: str = sys.platform) -> str:
    arch = project.Project().deb_arch
    testing = "(testing) " if _is_ci_env() else ""

    if platform == "linux":
        os_platform = _get_linux_release(os_release.OsRelease())
    else:
        os_platform = platform.title()

    return f"snapcraft/{snapcraft.__version__} {testing}{os_platform} ({arch})"
