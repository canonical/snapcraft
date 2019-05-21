# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018 Canonical Ltd
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

import logging
import os
import re

import snapcraft
from ._environment_checks import EnvironmentChecks


logger = logging.getLogger(__name__)


_EXPECTED_SNAP_DIR_PATTERNS = {
    re.compile(r"^snapcraft.yaml$"),
    re.compile(r"^.snapcraft(/state)?$"),
    re.compile(r"^hooks(/.*)?$"),
    re.compile(r"^local(/.*)?$"),
    re.compile(r"^plugins(/.*)?$"),
    re.compile(r"^gui(/.*\.(png|svg|desktop))?$"),
}


def conduct_project_sanity_check(project: snapcraft.project.Project):
    """Sanity check the project itself before continuing.

    The checks done here are meant to be light, and not rely on the build environment.
    """

    snap_dir_path = os.path.join(project._project_dir, "snap")
    if os.path.isdir(snap_dir_path):
        _check_snap_dir(snap_dir_path)


def get_project_environment_warnings(project: snapcraft.project.Project) -> str:
    return EnvironmentChecks(project).get_messages()


def _check_snap_dir(snap_dir_path: str):
    unexpected_paths = set()
    for root, directories, files in os.walk(snap_dir_path):
        for entry in directories + files:
            path = os.path.relpath(os.path.join(root, entry), snap_dir_path)
            if not _snap_dir_path_expected(path):
                unexpected_paths.add(path)

    if unexpected_paths:
        logger.warn(
            "The snap/ directory is meant specifically for snapcraft, but it contains "
            "the following non-snapcraft-related paths, which is unsupported and will "
            "cause unexpected behavior:"
            "\n- {}\n\n"
            "If you must store these files within the snap/ directory, move them to "
            "snap/local/, which is ignored by snapcraft.".format(
                "\n- ".join(sorted(unexpected_paths))
            )
        )


def _snap_dir_path_expected(path):
    for pattern in _EXPECTED_SNAP_DIR_PATTERNS:
        if pattern.match(path):
            return True
    return False
