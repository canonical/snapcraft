# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2016-2023 Canonical Ltd.
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

import functools
import os
import platform
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Set

from craft_cli import emit
from elftools.common.exceptions import ELFError

from . import ElfFile, errors


@functools.lru_cache(maxsize=1)
def get_elf_files(root_path: Path) -> List[ElfFile]:
    """Obtain a set of all ELF files in a subtree.

    :param root_path: The root of the subtree to list ELF files from.
    :return: A set of ELF files found in the given subtree.
    """
    file_list: List[str] = []
    for root, _, files in os.walk(str(root_path)):
        for file_name in files:
            # Filter out object files
            if file_name.endswith(".o"):
                continue

            file_path = os.path.join(root, file_name)
            if not os.path.islink(file_path):
                file_list.append(file_path)

    return get_elf_files_from_list(root_path, file_list)


def get_elf_files_from_list(root: Path, file_list: Iterable[str]) -> List[ElfFile]:
    """Return a list of ELF files from file_list prepended with root.

    :param str root: the root directory from where the file_list is generated.
    :param file_list: a list of file in root.
    :returns: a list of ElfFile objects.
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
        except ELFError:
            # Ignore invalid ELF files.
            continue
        except errors.CorruptedElfFile as exception:
            # Log if the ELF file seems corrupted
            emit.message(str(exception))
            continue

        # If ELF has dynamic symbols, add it.
        if elf_file.needed:
            elf_files.add(elf_file)

    return sorted(elf_files, key=lambda x: x.path)


@dataclass(frozen=True)
class _ArchConfig:
    arch_triplet: str
    dynamic_linker: str


_ARCH_CONFIG = {
    "aarch64": _ArchConfig("aarch64-linux-gnu", "lib/ld-linux-aarch64.so.1"),
    "armv7l": _ArchConfig("arm-linux-gnueabihf", "lib/ld-linux-armhf.so.3"),
    "ppc64le": _ArchConfig("powerpc64le-linux-gnu", "lib64/ld64.so.2"),
    "riscv64": _ArchConfig("riscv64-linux-gnu", "lib/ld-linux-riscv64-lp64d.so.1"),
    "s390x": _ArchConfig("s390x-linux-gnu", "lib/ld64.so.1"),
    "x86_64": _ArchConfig("x86_64-linux-gnu", "lib64/ld-linux-x86-64.so.2"),
}


def get_dynamic_linker(*, root_path: Path, snap_path: Path) -> str:
    """Obtain the dynamic linker that would be seen at runtime.

    :param root_path: The root path of a snap payload tree.
    :param snap_path: Absolute path to the snap once installed.

    :return: The path to the dynamic linker to use.
    """
    arch = platform.machine()
    arch_config = _ARCH_CONFIG.get(arch)
    if not arch_config:
        raise RuntimeError(f"Dynamic linker not defined for arch {arch!r}")

    linker_path = root_path / arch_config.dynamic_linker
    if not linker_path.exists():
        raise errors.DynamicLinkerNotFound(linker_path)

    return str(snap_path / arch_config.dynamic_linker)


def get_arch_triplet() -> str:
    """Inform the arch triplet string for the current architecture."""
    arch = platform.machine()
    arch_config = _ARCH_CONFIG.get(arch)
    if not arch_config:
        raise RuntimeError(f"Arch triplet not defined for arch {arch!r}")

    return arch_config.arch_triplet


def get_all_arch_triplets() -> List[str]:
    """Get a list of all architecture triplets."""
    return [architecture.arch_triplet for architecture in _ARCH_CONFIG.values()]
