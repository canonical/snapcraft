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

"""Snap file packing."""

import subprocess
from pathlib import Path
from typing import List, Optional, Union

from craft_cli import emit

from snapcraft import errors


def _verify_snap(directory: Path) -> None:
    emit.trace("pack_snap: check skeleton")
    try:
        subprocess.run(
            ["snap", "pack", "--check-skeleton", directory],
            capture_output=True,
            check=True,
            universal_newlines=True,
        )
    except subprocess.CalledProcessError as err:
        msg = f"Cannot pack snap file: {err!s}"
        if err.stderr:
            msg += f" ({err.stderr.strip()!s})"
        raise errors.SnapcraftError(msg)


def _get_directory(output: Optional[str]) -> Optional[str]:
    """Get directory to output the snap file to.

    :param output: Snap file name or directory.

    :return: The directory to output the snap file to.
      If no directory is provided, return None.
      If directory is current working directory, return None.
    """
    if output:
        output_path = Path(output)
        output_parent = output_path.parent
        if output_path.is_dir():
            return str(output_path)
        if output_parent and output_parent.resolve() != Path(".").resolve():
            return str(output_parent)
    return None


def _get_filename(
    output: Optional[str],
    name: Optional[str] = None,
    version: Optional[str] = None,
    target_arch: Optional[str] = None,
) -> Optional[str]:
    """Get output filename of the snap file.

    If output is not a file, then the filename will be <name>_<version>_<target_arch>.snap

    :param output: Snap file name or directory.
    :param name: Name of snap project.
    :param version: Version of snap project.
    :param target_arch: Target architecture the snap project is built to.

    :return: The filename of the snap file.
    """
    if name is not None and version is not None and target_arch is not None:
        output_file = name + "_" + version + "_" + target_arch + ".snap"
    else:
        output_file = None

    if output:
        output_path = Path(output)
        if not output_path.is_dir():
            output_file = output_path.name

    return output_file


def pack_snap(
    directory: Path,
    *,
    output: Optional[str],
    compression: Optional[str] = None,
    name: Optional[str] = None,
    version: Optional[str] = None,
    target_arch: Optional[str] = None,
) -> None:
    """Pack snap contents.

    :param directory: Directory to pack.
    :param output: Snap file name or directory.
    :param compression: Compression type to use, None for defaults.
    :param name: Name of snap project.
    :param version: Version of snap project.
    :param target_arch: Target architecture the snap project is built to.
    """
    emit.trace(f"pack_snap: output={output!r}, compression={compression!r}")

    # TODO remove workaround once LP: #1950465 is fixed
    _verify_snap(directory)

    command: List[Union[str, Path]] = ["snap", "pack"]
    output_file = _get_filename(output, name, version, target_arch)
    if output_file is not None:
        command.extend(["--filename", output_file])

    # When None, just use snap pack's default settings.
    if compression is not None:
        command.extend(["--compression", compression])

    command.append(directory)

    output_dir = _get_directory(output)
    if output_dir is not None:
        command.append(output_dir)

    emit.progress("Creating snap package...")
    emit.trace(f"Pack command: {command}")
    try:
        proc = subprocess.run(
            command, capture_output=True, check=True, universal_newlines=True
        )
    except subprocess.CalledProcessError as err:
        msg = f"Cannot pack snap file: {err!s}"
        if err.stderr:
            msg += f" ({err.stderr.strip()!s})"
        raise errors.SnapcraftError(msg)

    snap_filename = str(proc.stdout).partition(":")[2].strip()
    emit.message(f"Created snap package {snap_filename}", intermediate=True)
