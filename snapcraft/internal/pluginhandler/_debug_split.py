# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright (C) 2020 Canonical Ltd
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
import shlex
import subprocess
from pathlib import Path
from typing import List, Optional, Set, Tuple

from snapcraft import file_utils
from snapcraft.internal import errors
from snapcraft.internal.elf import ElfFile

logger = logging.getLogger(__name__)


def _cross_command(cmd: str, arch_triplet: Optional[str]) -> str:
    if arch_triplet:
        cmd = f"{arch_triplet}-{cmd}"
    return file_utils.get_tool_path(cmd)


def _run(cmd: List[str]) -> None:
    command_string = " ".join([shlex.quote(c) for c in cmd])
    logger.debug(f"running: {command_string}")

    try:
        subprocess.check_call(cmd)
    except subprocess.CalledProcessError as call_error:
        raise errors.SnapcraftCommandError(
            command=command_string, call_error=call_error
        ) from call_error


class DebugSplitter:
    def __init__(self, *, arch_triplet: str, debug_dir: Path) -> None:
        self.objcopy_cmd = _cross_command("objcopy", arch_triplet)
        self.strip_cmd = _cross_command("strip", arch_triplet)
        self.debug_dir = debug_dir

    def _get_debug_file_path(self, build_id: str) -> Path:
        return Path(self.debug_dir, build_id[:2], build_id[2:])

    def _make_debug(self, elf_file: ElfFile, debug_file: str) -> None:
        _run(
            [
                self.objcopy_cmd,
                "--only-keep-debug",
                "--compress-debug-sections",
                elf_file.path,
                debug_file,
            ]
        )

    def _attach_debug(self, elf_file: ElfFile, debug_file: str) -> None:
        _run([self.objcopy_cmd, "--add-gnu-debuglink", debug_file, elf_file.path])

    def _strip_debug_command(self, elf_file: ElfFile) -> None:
        # dh_strip use 0o111 to verify executable:
        # https://github.com/Debian/debhelper/blob/423cfce04719f41d7224d75155c4e7f9a97a10e9/dh_strip#L229
        if os.stat(elf_file.path).st_mode & 0o111 == 0o111:
            # Executable.
            cmd = [
                self.strip_cmd,
                "--remove-section=.comment",
                "--remove-section=.note",
                elf_file.path,
            ]
        else:
            # Shared object.
            cmd = [
                self.strip_cmd,
                "--remove-section=.comment",
                "--remove-section=.note",
                "--strip-unneeded",
                elf_file.path,
            ]

        _run(cmd)

    def split(self, elf_file: ElfFile) -> Optional[Path]:
        # Matching the pattern used in dh_strip:
        # https://github.com/Debian/debhelper/blob/master/dh_strip#L359

        if not elf_file.has_debug_info:
            logger.debug(f"No debug info found for {elf_file.path!r}.")
            return None

        if not elf_file.build_id:
            logger.debug(
                f"No debug info extracted for {elf_file.path!r} due to missing build-id."
            )
            return None

        if elf_file.elf_type not in ["ET_EXEC", "ET_DYN"]:
            logger.warning(
                f"Skipping debug extraction for {elf_file.path!r} with ELF type {elf_file.elf_type!r}"
            )
            return None

        debug_file = self._get_debug_file_path(elf_file.build_id)
        debug_file.parent.mkdir(exist_ok=True, parents=True)

        # Copy debug information to debug directory.
        self._make_debug(elf_file, debug_file.as_posix())

        # Strip debug from binary.
        self._strip_debug_command(elf_file)

        # Link/attach debug symbol file to executable.
        self._attach_debug(elf_file, debug_file.as_posix())

        return debug_file


def split_debug_info(
    *, debug_dir: str, arch_triplet: str, prime_dir: str, file_paths: Set[str]
) -> Tuple[Set[str], Set[str]]:
    """Split files in file_paths, saving debug artifacts to debug_dir.

    :param debug_dir: Directory to save artifacts to.
    :param arch_triplet: Project's arch triplet, used to determine which
           objcopy and strip command to use.
    :param file_paths: Set of file paths (string) to process.
    :return: a tuple of the set of (debug_files, debug_dirs) artifacts.
    """

    debug_files: Set[str] = set()
    debug_dirs: Set[str] = set()

    logger.warning(f"Collected debug information may be found in: {debug_dir}")

    collector = DebugSplitter(arch_triplet=arch_triplet, debug_dir=Path(debug_dir))
    for file_path in file_paths:
        file_path = os.path.join(prime_dir, file_path)
        if not ElfFile.is_elf(file_path):
            continue

        logger.debug(f"Checking debug info for: {file_path!r}")

        debug_file = collector.split(ElfFile(path=file_path))
        if debug_file is None:
            continue

        debug_file = debug_file.relative_to(debug_dir)
        debug_files.add(debug_file.as_posix())
        debug_dirs.add(debug_file.parent.as_posix())

    return debug_files, debug_dirs
