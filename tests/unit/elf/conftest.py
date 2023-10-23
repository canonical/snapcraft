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

import os
import shutil
from pathlib import Path

import pytest

from snapcraft.elf import ElfFile, _elf_file
from tests import TESTS_DIR


@pytest.fixture
def fake_elf(mocker, new_dir):
    def _fake_elf(name: str) -> ElfFile:
        Path(name).write_bytes(b"\x7fELF")
        if name.endswith("fake_elf-bad-patchelf"):
            with Path(name).open("ab") as f:
                f.write(b"nointerpreter")
        return ElfFile(path=Path(name))

    mocker.patch(
        "snapcraft.elf._elf_file.ElfFile._extract_attributes",
        new=_fake_elffile_extract_attributes,
    )
    yield _fake_elf


@pytest.fixture
def fake_libs(new_dir):
    core_path = new_dir / "core"
    core_path.mkdir()

    root_libraries = {
        "foo.so.1": new_dir / "foo.so.1",
        "moo.so.2": new_dir / "non-standard" / "moo.so.2",
    }

    barsnap_elf = core_path / "barsnap.so.2"
    elf_list = [*root_libraries.values(), barsnap_elf]

    for lib in elf_list:
        lib.parent.mkdir(exist_ok=True)
        lib.write_bytes(b"\x7fELF")

    yield root_libraries


@pytest.fixture
def fake_tools(new_dir, monkeypatch):
    bin_skel_path = TESTS_DIR / "bin" / "elf"
    bin_path = new_dir / "bin"
    bin_path.mkdir(exist_ok=True)

    # Copy strip and execstack
    for tool in ["strip", "execstack"]:
        src_path = bin_skel_path / tool
        dest_path = bin_path / tool
        shutil.copy(src_path, dest_path)
        dest_path.chmod(0o755)

    # Some values in ldd need to be set with core_path
    core_path = new_dir / "core"
    src_ldd_path = bin_skel_path / "ldd"
    dest_ldd_path = bin_path / "ldd"
    with src_ldd_path.open() as rf, dest_ldd_path.open("w") as wf:
        for line in rf.readlines():
            wf.write(line.replace("{CORE_PATH}", str(core_path)))
    dest_ldd_path.chmod(0o755)

    # Set version in fake patchelf
    src_patchelf_path = bin_skel_path / "patchelf"
    dest_patchelf_path = bin_path / "patchelf"
    with src_patchelf_path.open() as rf, dest_patchelf_path.open("w") as wf:
        for line in rf.readlines():
            wf.write(line.replace("{VERSION}", "0.14.3"))
    dest_patchelf_path.chmod(0o755)

    monkeypatch.setenv("PATH", f"{bin_path!s}:{os.getenv('PATH')}")


def _fake_elffile_extract_attributes(self):  # pylint: disable=too-many-statements  # noqa: PLR0915
    """Mock method definition for ElfFile._extract_attributes()."""
    name = self.path.name

    self.arch_tuple = ("ELFCLASS64", "ELFDATA2LSB", "EM_X86_64")
    self.build_id = f"build-id-{name}"

    if name in [
        "fake_elf-2.26",
        "fake_elf-bad-ldd",
        "fake_elf-with-core-libs",
        "fake_elf-with-missing-libs",
        "fake_elf-bad-patchelf",
        "fake_elf-with-host-libraries",
    ]:
        glibc = _elf_file._NeededLibrary(name="libc.so.6")
        glibc.add_version("GLIBC_2.2.5")
        glibc.add_version("GLIBC_2.26")

        self.interp = "/lib64/ld-linux-x86-64.so.2"
        self.soname = ""
        self.versions = set()
        self.needed = {glibc.name: glibc}
        self.execstack_set = False
        self.is_dynamic = True
        self.has_debug_info = False

    elif name == "fake_elf-2.23":
        glibc = _elf_file._NeededLibrary(name="libc.so.6")
        glibc.add_version("GLIBC_2.2.5")
        glibc.add_version("GLIBC_2.23")

        self.interp = "/lib64/ld-linux-x86-64.so.2"
        self.soname = ""
        self.versions = set()
        self.needed = {glibc.name: glibc}
        self.execstack_set = False
        self.is_dynamic = True
        self.has_debug_info = False

    elif name == "fake_elf-1.1":
        glibc = _elf_file._NeededLibrary(name="libc.so.6")
        glibc.add_version("GLIBC_1.1")
        glibc.add_version("GLIBC_0.1")

        self.interp = "/lib64/ld-linux-x86-64.so.2"
        self.soname = ""
        self.versions = set()
        self.needed = {glibc.name: glibc}
        self.execstack_set = False
        self.is_dynamic = True
        self.has_debug_info = False

    elif name == "fake_elf-static":
        self.interp = "/lib64/ld-linux-x86-64.so.2"
        self.soname = ""
        self.versions = set()
        self.needed = {}
        self.execstack_set = False
        self.is_dynamic = False
        self.has_debug_info = False

    elif name == "fake_elf-shared-object":
        openssl = _elf_file._NeededLibrary(name="libssl.so.1.0.0")
        openssl.add_version("OPENSSL_1.0.0")

        self.interp = ""
        self.soname = "libfake_elf.so.0"
        self.versions = set()
        self.needed = {openssl.name: openssl}
        self.execstack_set = False
        self.is_dynamic = True
        self.has_debug_info = False

    elif name == "fake_elf-with-execstack":
        glibc = _elf_file._NeededLibrary(name="libc.so.6")
        glibc.add_version("GLIBC_2.23")

        self.interp = "/lib64/ld-linux-x86-64.so.2"
        self.soname = ""
        self.versions = set()
        self.needed = {glibc.name: glibc}
        self.execstack_set = True
        self.is_dynamic = True
        self.has_debug_info = False

    elif name == "fake_elf-with-bad-execstack":
        glibc = _elf_file._NeededLibrary(name="libc.so.6")
        glibc.add_version("GLIBC_2.23")

        self.interp = "/lib64/ld-linux-x86-64.so.2"
        self.soname = ""
        self.versions = set()
        self.needed = {glibc.name: glibc}
        self.execstack_set = True
        self.is_dynamic = True
        self.has_debug_info = False

    elif name == "libc.so.6":
        self.interp = ""
        self.soname = "libc.so.6"
        self.versions = {"libc.so.6", "GLIBC_2.2.5", "GLIBC_2.23", "GLIBC_2.26"}
        self.needed = {}
        self.execstack_set = False
        self.is_dynamic = True
        self.has_debug_info = False

    elif name == "libssl.so.1.0.0":
        self.interp = ""
        self.soname = "libssl.so.1.0.0"
        self.versions = {"libssl.so.1.0.0", "OPENSSL_1.0.0"}
        self.needed = {}
        self.execstack_set = False
        self.is_dynamic = True
        self.has_debug_info = False

    else:
        self.interp = ""
        self.soname = ""
        self.versions = set()
        self.needed = {}
        self.execstack_set = False
        self.is_dynamic = True
        self.has_debug_info = False
