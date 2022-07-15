# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2022 Canonical Ltd.
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

"""Helpers to handle ELF files."""

import os
import platform
from pathlib import Path
from typing import FrozenSet, Set

import elftools.common.exceptions
import elftools.elf.elffile
from craft_cli import emit

from snapcraft.errors import SnapcraftError

from . import ElfFile, errors


def get_elf_files(root: str, file_list: Set[str]) -> FrozenSet[ElfFile]:
    """Return a frozenset of ELF files from file_list prepended with root.

    :param str root: the root directory from where the file_list is generated.
    :param file_list: a list of file in root.
    :returns: a frozentset of ElfFile objects.
    """
    elf_files: Set[ElfFile] = set()

    for part_file in file_list:
        # Filter out object (*.o) files-- we only care about binaries.
        if part_file.endswith(".o"):
            continue

        # No need to crawl links-- the original should be here, too.
        path = Path(root, part_file)
        if os.path.islink(path):
            emit.debug(f"Skipped link {path!r} while finding dependencies")
            continue

        # Ignore if file does not have ELF header.
        if not ElfFile.is_elf(path):
            continue

        try:
            elf_file = ElfFile(path=path)
        except elftools.common.exceptions.ELFError:
            # Ignore invalid ELF files.
            continue
        except errors.CorruptedElfFile as exception:
            # Log if the ELF file seems corrupted
            emit.message(str(exception))
            continue

        # If ELF has dynamic symbols, add it.
        if elf_file.needed:
            elf_files.add(elf_file)

    return frozenset(elf_files)


_PLATFORM_DYNAMIC_LINKER = {
    "aarch64": "lib/ld-linux-aarch64.so.1",
    "armv7l": "lib/ld-linux-armhf.so.3",
    "ppc64le": "lib64/ld64.so.2",
    "riscv64": "lib/ld-linux-riscv64-lp64d.so.1",
    "s390x": "lib/ld64.so.1",
    "x86_64": "lib64/ld-linux-x86-64.so.2",
}


def get_dynamic_linker(*, root_path: Path, snap_path: Path) -> str:
    """Obtain the dynamic linker that would be seen at runtime.

    :param root_path: The root path of a snap payload tree.
    :param snap_path: Absolute path to the snap once installed.

    :return: The path to the dynamic linker to use.
    """
    arch = platform.machine()
    linker = _PLATFORM_DYNAMIC_LINKER.get(arch)
    if not linker:
        raise RuntimeError(f"Dynamic linker not defined for arch {arch!r}")

    linker_path = root_path / linker
    if not linker_path.exists():
        raise SnapcraftError(f"Dynamic linker {str(linker_path)!r} not found.")

    return str(snap_path / linker)
