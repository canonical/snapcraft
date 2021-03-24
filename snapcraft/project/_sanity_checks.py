# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2018-2019 Canonical Ltd
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

from snapcraft.internal import deprecations
from snapcraft.internal.errors import SnapcraftEnvironmentError
from snapcraft.project import Project

logger = logging.getLogger(__name__)


_EXPECTED_SNAP_DIR_PATTERNS = {
    re.compile(r"^snapcraft.yaml$"),
    re.compile(r"^.snapcraft([/|\\]state)?$"),
    re.compile(r"^hooks([/|\\].*)?$"),
    re.compile(r"^keys([/|\\].*\.(asc))?$"),
    re.compile(r"^local([/|\\].*)?$"),
    re.compile(r"^plugins([/|\\].*)?$"),
    re.compile(r"^gui([/|\\].*\.(png|svg|desktop))?$"),
}


def conduct_project_sanity_check(project: Project, **kwargs) -> None:
    """Sanity check the project itself before continuing.

    The checks done here are meant to be light, and not rely on the build environment.
    """
    # The snapcraft.yaml should be valid even without extensions applied
    # This here check is mostly for backwards compatibility with the
    # rest of the code base.
    if project.info is not None:
        project.info.validate_raw_snapcraft()

    if project._get_build_base() == "core":
        deprecations.handle_deprecation_notice("dn13")

    snap_dir_path = os.path.join(project._get_snapcraft_assets_dir())
    if os.path.isdir(snap_dir_path):
        # TODO: move this check to the ProjectInfo class.
        _check_snap_dir(snap_dir_path)

    if (
        project._get_build_base() in ["core20"]
        and kwargs.get("target_arch") is not None
        and not os.getenv("SNAPCRAFT_ENABLE_EXPERIMENTAL_TARGET_ARCH")
    ):
        raise SnapcraftEnvironmentError(
            "*EXPERIMENTAL* '--target-arch' configured, but not enabled. "
            "Enable with '--enable-experimental-target-arch' flag."
        )

    # Icon should refer to project file, verify it exists.
    icon = project.info.get_raw_snapcraft().get("icon")
    if icon and not os.path.exists(icon):
        raise SnapcraftEnvironmentError(f"Specified icon {icon!r} does not exist.")


def _check_snap_dir(snap_dir_path: str) -> None:
    unexpected_paths = set()
    for root, directories, files in os.walk(snap_dir_path):
        for entry in directories + files:
            path = os.path.relpath(os.path.join(root, entry), snap_dir_path)
            if not _snap_dir_path_expected(path):
                unexpected_paths.add(path)

    if unexpected_paths:
        snap_dir_relpath = os.path.relpath(snap_dir_path, os.getcwd())
        logger.warning(
            "The {snap_dir!r} directory is meant specifically for snapcraft, but it contains "
            "the following non-snapcraft-related paths, which is unsupported and will "
            "cause unexpected behavior:"
            "\n- {unexpected_files}\n\n"
            "If you must store these files within the {snap_dir!r} directory, move them to "
            "{snap_dir_local!r}, which is ignored by snapcraft.".format(
                snap_dir=snap_dir_relpath,
                snap_dir_local=os.path.join(snap_dir_relpath, "local"),
                unexpected_files="\n- ".join(sorted(unexpected_paths)),
            )
        )


def _snap_dir_path_expected(path: str) -> bool:
    for pattern in _EXPECTED_SNAP_DIR_PATTERNS:
        if pattern.match(path):
            return True
    return False
