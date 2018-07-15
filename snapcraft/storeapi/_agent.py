# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2017 Canonical Ltd
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
import platform

import snapcraft


def _is_ci_env():
    env_prefixes = ["TRAVIS", "AUTOPKGTEST_TMP"]
    matches = []

    for prefix in env_prefixes:
        matches += [var for var in os.environ.keys() if var.startswith(prefix)]

    return len(matches) > 0


def get_user_agent():
    arch = snapcraft.ProjectOptions().deb_arch
    testing = "(testing) " if _is_ci_env() else ""
    return "snapcraft/{} {}{} ({})".format(
        snapcraft.__version__,
        testing,
        "/".join(platform.dist()[0:2]),  # i.e. Ubuntu/16.04
        arch,
    )
