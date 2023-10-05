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

import shutil
import stat
from functools import partial
from hashlib import md5
from pathlib import Path
from typing import List


def get_build_id(app_name: str, project_name: str, project_path: Path) -> str:
    """Get the build id for a project.

    The build id is formatted as `snapcraft-<project-name>-<hash>`.
    The hash is a hash of all files in the project directory.

    :param app_name: Name of the application.
    :param project_name: Name of the project.
    :param project_path: Path of the project.

    :returns: The build id.
    """
    project_hash = _compute_hash(project_path)

    return f"{app_name}-{project_name}-{project_hash}"


def _compute_hash(directory: Path) -> str:
    """Compute an md5 hash from the contents of the files in a directory.

    If a file or its contents within the directory are modified, then the hash
    will be different.

    The hash may not be unique if the contents of one file are moved to another file
    or if files are reorganized.

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

    files = sorted([file for file in Path().glob("**/*") if file.is_file()])
    hashes: List[str] = []

    for file_path in files:
        md5_hash = md5()  # noqa: S324 (insecure-hash-function)
        with open(file_path, "rb") as file:
            # read files in chunks in case they are large
            for block in iter(partial(file.read, 4096), b""):
                md5_hash.update(block)
        hashes.append(md5_hash.hexdigest())

    all_hashes = "".join(hashes).encode()
    return md5(all_hashes).hexdigest()  # noqa: S324 (insecure-hash-function)


def rmtree(directory: Path) -> None:
    """Cross-platform rmtree implementation.

    :param directory: Directory to remove.
    """
    shutil.rmtree(
        str(directory.resolve()),
        onerror=_remove_readonly,  # type: ignore
    )


def _remove_readonly(func, filepath, _):
    """Shutil onerror function to make read-only files writable.

    Try setting file to writeable if error occurs during rmtree. Known to be required
    on Windows where file is not writeable, but it is owned by the user (who can
    set file permissions).

    :param filepath: filepath to make writable
    """
    Path(filepath).chmod(stat.S_IWRITE)
    func(filepath)
