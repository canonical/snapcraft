# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2022,2024 Canonical Ltd.
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
    emit.debug("pack_snap: check skeleton")
    try:
        subprocess.run(
            ["snap", "pack", "--check-skeleton", directory],
            capture_output=True,
            check=True,
            universal_newlines=True,
        )
    except subprocess.CalledProcessError as err:
        stderr = None
        if err.stderr:
            stderr = err.stderr.strip()
            msg = f"Cannot pack snap: {stderr!s}"
        else:
            msg = "Cannot pack snap"
        raise errors.SnapcraftError(msg, details=f"{err!s}") from err


def _get_directory(output: Optional[str]) -> Path:
    """Get directory to output the snap file to.

    If no directory is provided, return current working directory.

    :param output: Snap output file name or directory.

    :return: The directory to output the snap file to.
    """
    if output:
        output_path = Path(output)
        if output_path.is_dir():
            return output_path.resolve()
        return output_path.parent.resolve()

    return Path.cwd()


def _get_filename(
    output: Optional[str],
    name: Optional[str] = None,
    version: Optional[str] = None,
    target_arch: Optional[str] = None,
) -> Optional[str]:
    """Get output filename of the snap file.

    If `output` is not a file, then the filename will be <name>_<version>_<target_arch>.snap

    :param output: Snap file name or directory.
    :param name: Name of snap project.
    :param version: Version of snap project.
    :param target_arch: Target architecture the snap project is built to.

    :return: The filename of the snap file if output or name/version/target_arch are specified.
    """
    if output:
        output_path = Path(output)
        if not output_path.is_dir():
            return output_path.name

    if all(i is not None for i in [name, version, target_arch]):
        return f"{name}_{version}_{target_arch}.snap"

    return None


def _pack(
    directory: Path,
    output_dir: Path,
    output_file: Optional[str],
    compression: Optional[str],
) -> str:
    """Pack a directory with `snap pack` as a snap or component.

    :param directory: Directory to pack.
    :param output_dir: Directory to output the artefact to.
    :param output_file: Name of the artefact.
    :param compression: Compression type to use, None for default.

    :returns: The filename of the packed snap or component.

    :raises SnapcraftError: If the directory cannot be packed.
    """
    command: List[Union[str, Path]] = ["snap", "pack"]

    if output_file:
        command.extend(["--filename", output_file])
    if compression:
        command.extend(["--compression", compression])
    command.extend([directory, output_dir])

    emit.debug(f"Pack command: {command}")
    try:
        proc = subprocess.run(
            command, capture_output=True, check=True, universal_newlines=True
        )
    except subprocess.CalledProcessError as err:
        msg = str(err)
        details = None
        if err.stderr:
            details = err.stderr.strip()
        raise errors.SnapcraftError(msg, details=details) from err

    return Path(str(proc.stdout).partition(":")[2].strip()).name


def pack_component(
    directory: Path, output_dir: Path, compression: Optional[str] = None
) -> str:
    """Pack a directory containing component data.

    Calls `snap pack <options> <component-dir> <output-dir>`.

    Requires snapd to be installed from the `latest/edge` channel.

    :param directory: Directory to pack.
    :param compression: Compression type to use, None for default.
    :param output_dir: Directory to output component to.

    :returns: The filename of the packed component.

    :raises SnapcraftError: If the component cannot be packed.
    """
    try:
        return _pack(
            directory=directory,
            output_dir=output_dir,
            output_file=None,
            compression=compression,
        )
    except errors.SnapcraftError as err:
        err.resolution = (
            "Packing components is experimental and requires `snapd` "
            "to be installed from the `latest/edge` channel."
        )
        raise


def pack_snap(
    directory: Path,
    *,
    output: Optional[str],
    compression: Optional[str] = None,
    name: Optional[str] = None,
    version: Optional[str] = None,
    target_arch: Optional[str] = None,
) -> str:
    """Pack snap contents with `snap pack`.

    Calls `snap pack <options> <snap-dir> <output-dir>`.

    `output` may either be a directory, a file path, or just a file name.
      - directory: write snap to directory with default snap name
      - file path: write snap to specified directory with specified snap name
      - file name: write snap to cwd with specified snap name

    If name, version, and target architecture are not specified, then snap
    will use its default naming convention.

    :param directory: Directory to pack.
    :param output: Snap file name or directory.
    :param compression: Compression type to use, None for defaults.
    :param name: Name of snap project.
    :param version: Version of snap project.
    :param target_arch: Target architecture the snap project is built to.

    :returns: The filename of the packed snap.

    :raises SnapcraftError: If the directory cannot be packed.
    """
    emit.debug(f"pack_snap: output={output!r}, compression={compression!r}")

    # TODO remove workaround once LP: #1950465 is fixed
    _verify_snap(directory)

    output_dir = _get_directory(output)
    output_file = _get_filename(output, name, version, target_arch)

    emit.progress("Creating snap package...")
    return _pack(
        directory=directory,
        output_dir=output_dir,
        output_file=output_file,
        compression=compression,
    )
