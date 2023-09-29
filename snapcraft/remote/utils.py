# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2023 Canonical Ltd.
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

"""Remote build utilities."""

from functools import partial
from hashlib import md5
from pathlib import Path


def get_build_id(app_name: str, project_name: str, project_path: Path) -> str:
    """Get the build id for a project.

    The build id is formatted as `snapcraft-<project-name>-<hash>`.
    The hash is a hash of all files in the project directory.

    :returns: The build id.

    :raises SnapcraftError: If the snapcraft.yaml does not contain a 'name' keyword.
    """
    project_hash = _compute_hash(project_path)

    return f"{app_name}-{project_name}-{project_hash}"


def _compute_hash(directory: Path) -> str:
    """Compute an md5 hash from the contents of the files in a directory.

    If a file or its contents within the directory are modified, then the hash
    will be different.

    :returns: A string containing the md5 hash.

    :raises FileNotFoundError: If the path is not a directory or does not exist.
    """
    if not directory.exists():
        raise FileNotFoundError(
            f"Could not compute hash because directory {str(directory.absolute())} "
            "does not exist."
        )
    if not directory.is_dir():
        raise FileNotFoundError(
            f"Could not compute hash because {str(directory.absolute())} is not "
            "a directory."
        )

    # sort the list of files for reproducibility
    files = sorted([file for file in directory.glob("**/*") if file.is_file()])
    md5_hash = md5()  # noqa: S324 (insecure-hash-function)

    for file_path in files:
        with open(file_path, "rb") as file:
            # read files in chunks in case they are large
            for block in iter(partial(file.read, 4096), b""):
                md5_hash.update(block)

    return md5_hash.hexdigest()
