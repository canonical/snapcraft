# -*- Mode:Python; indent-tabs-mode:nil; tab-width:4 -*-
#
# Copyright 2026 Canonical Ltd.
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

"""Integration tests for the library linter using ELF binaries."""

import struct
from pathlib import Path

import pytest

from snapcraft import linters, models
from snapcraft.elf import elf_utils
from snapcraft.meta import snap_yaml


def _write_foreign_elf(path: Path) -> None:
    """Write a minimal 32-bit big-endian PowerPC ELF file.

    EM_PPC is foreign on every supported host arch, so the linter must
    skip it without invoking it via binfmt/QEMU.
    """
    elf_header = b"\x7fELF" + bytes([1, 2, 1, 0]) + b"\x00" * 8
    elf_header += struct.pack(
        ">HHIIIIIHHHHHH",
        2,  # e_type = ET_EXEC
        20,  # e_machine = EM_PPC
        1,  # e_version
        0,  # e_entry
        0,  # e_phoff
        0,  # e_shoff
        0,  # e_flags
        52,  # e_ehsize
        0,
        0,  # e_phentsize, e_phnum
        0,
        0,  # e_shentsize, e_shnum
        0,  # e_shstrndx
    )
    path.write_bytes(elf_header)


@pytest.fixture(autouse=True)
def _clear_elf_cache():
    elf_utils.get_elf_files.cache_clear()


def test_library_linter_skips_foreign_arch_elf(new_dir):
    """The library linter must not call load_dependencies() on a foreign-arch ELF.

    A minimal powerpc (EM_PPC) ELF is placed in the prime directory.
    On every supported host the linter should silently skip it and report
    no library issues, rather than trying to invoke it via binfmt/QEMU.
    """
    _write_foreign_elf(Path("bash-powerpc"))

    yaml_data = {
        "name": "mytest",
        "version": "1.0",
        "base": "core22",
        "summary": "Foreign-arch ELF linter integration test",
        "description": "test",
        "confinement": "strict",
        "parts": {},
    }
    project = models.Project.unmarshal(yaml_data)
    snap_yaml.write(project, prime_dir=Path(new_dir), arch="amd64")

    issues = linters.run_linters(new_dir, lint=None)

    library_issues = [i for i in issues if i.name == "library"]
    assert library_issues == []
