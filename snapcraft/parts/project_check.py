# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022 Canonical Ltd.
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

"""Verify expected files in payload."""

import os
import re
from pathlib import Path

from craft_cli import emit

from snapcraft import errors
from snapcraft.projects import Project

_EXPECTED_SNAP_DIR_PATTERNS = {
    re.compile(r"^snapcraft.yaml$"),
    re.compile(r"^.snapcraft([/|\\]state)?$"),
    re.compile(r"^hooks([/|\\].*)?$"),
    re.compile(r"^keys([/|\\].*\.(asc))?$"),
    re.compile(r"^local([/|\\].*)?$"),
    re.compile(r"^plugins([/|\\].*)?$"),
    re.compile(r"^gui([/|\\].*\.(png|svg|desktop))?$"),
}


def run_project_checks(project: Project, *, assets_dir: Path) -> None:
    """Execute consistency checks for project and project files.

    The checks done here are meant to be light, and not rely on the
    build environment.
    """
    # Assets dir shouldn't contain unexpected files.
    if assets_dir.is_dir():
        _check_snap_dir(assets_dir)

    # Icon should refer to project file, verify it exists.
    if project.icon and not Path(project.icon).exists():
        raise errors.SnapcraftError(f"Specified icon {project.icon!r} does not exist.")


def _check_snap_dir(snap_dir_path: Path) -> None:
    """Verify if the given path only contains expected files."""
    unexpected_paths = set()
    for root, directories, files in os.walk(snap_dir_path):
        for entry in directories + files:
            path = Path(root, entry)
            relpath = path.relative_to(snap_dir_path)
            if not _snap_dir_path_expected(relpath):
                unexpected_paths.add(str(relpath))

    if unexpected_paths:
        snap_dir_relpath = snap_dir_path.relative_to(Path())
        emit.message(
            "The {snap_dir!r} directory is meant specifically for snapcraft, but it contains\n"
            "the following non-snapcraft-related paths:"
            "\n- {unexpected_files}\n\n"
            "This is unsupported and may cause unexpected behavior. If you must store\n"
            "these files within the {snap_dir!r} directory, move them to {snap_dir_local!r}\n"
            "which is ignored by snapcraft.".format(
                snap_dir=str(snap_dir_relpath),
                snap_dir_local=str(snap_dir_relpath / "local"),
                unexpected_files="\n- ".join(sorted(unexpected_paths)),
            ),
            intermediate=True
        )


def _snap_dir_path_expected(path: Path) -> bool:
    """Verify if the given path matches one of the expected patterns."""
    for pattern in _EXPECTED_SNAP_DIR_PATTERNS:
        if pattern.match(str(path)):
            return True
    return False
