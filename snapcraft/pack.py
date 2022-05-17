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


def pack_snap(
    directory: Path, *, output: Optional[str], compression: Optional[str] = None
) -> None:
    """Pack snap contents.

    :param directory: Directory to pack.
    :param output: Snap file name or directory.
    :param compression: Compression type to use, None for defaults.
    """
    emit.trace(f"pack_snap: output={output!r}, compression={compression!r}")

    # TODO remove workaround once LP: #1950465 is fixed
    _verify_snap(directory)

    output_file = None
    output_dir = None

    if output:
        output_path = Path(output)
        output_parent = output_path.parent
        if output_path.is_dir():
            output_dir = str(output_path)
        elif output_parent and output_parent != Path("."):
            output_dir = str(output_parent)
            output_file = output_path.name
        else:
            output_file = output

    command: List[Union[str, Path]] = ["snap", "pack"]
    if output_file is not None:
        command.extend(["--filename", output_file])

    # When None, just use snap pack's default settings.
    if compression is not None:
        command.extend(["--compression", compression])

    command.append(directory)

    if output_dir is not None:
        command.append(output_dir)

    emit.progress("Creating snap package...")
    emit.trace(f"Pack command: {command}")
    try:
        subprocess.run(
            command, capture_output=True, check=True, universal_newlines=True
        )  # type: ignore
    except subprocess.CalledProcessError as err:
        msg = f"Cannot pack snap file: {err!s}"
        if err.stderr:
            msg += f" ({err.stderr.strip()!s})"
        raise errors.SnapcraftError(msg)

    emit.message("Created snap package", intermediate=True)
